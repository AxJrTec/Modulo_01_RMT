"""
canny_to_points.py
------------------
Extrae los bordes Canny de una imagen y genera una lista de puntos ordenados
mediante Nearest Neighbor (TSP greedy) para que una sola línea recorra
toda la imagen de forma continua.

Exporta en formato:  x1,y1; x2,y2; x3,y3; ...

Uso:
  python canny_to_points.py ken.jpg --visualize
  python canny_to_points.py ken.jpg --min-points 200 --max-points 300
  python canny_to_points.py ken.jpg --output puntos.txt
"""

import cv2
import numpy as np
import argparse
from pathlib import Path


# ── Parámetros por defecto ───────────────────────────────────────────────────
DEFAULT_MIN_POINTS  = 200
DEFAULT_MAX_POINTS  = 1400
DEFAULT_CANNY_LOW   = 50
DEFAULT_CANNY_HIGH  = 150
DEFAULT_EPSILON_REL = 0.0001
DEFAULT_STEP_PX     = 0.5


def load_image(path: str) -> np.ndarray:
    img = cv2.imread(path)
    if img is None:
        raise FileNotFoundError(f"No se pudo cargar: {path}")
    img = cv2.resize(img, (128, 235), interpolation=cv2.INTER_AREA)
    return img


def detect_edges(img: np.ndarray, low: int, high: int) -> np.ndarray:
    gray    = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges   = cv2.Canny(blurred, low, high)
    return edges


def sample_segment(p1: np.ndarray, p2: np.ndarray, step: float) -> list:
    vec  = p2 - p1
    dist = np.linalg.norm(vec)
    if dist == 0:
        return [(int(p1[0]), int(p1[1]))]
    n_steps = max(1, int(np.ceil(dist / step)))
    points  = []
    for i in range(n_steps + 1):
        t  = i / n_steps
        pt = p1 + t * vec
        points.append((int(round(pt[0])), int(round(pt[1]))))
    return points


def extract_points(edges: np.ndarray,
                   min_points: int    = DEFAULT_MIN_POINTS,
                   max_points: int    = DEFAULT_MAX_POINTS,
                   step_px: float     = DEFAULT_STEP_PX,
                   epsilon_rel: float = DEFAULT_EPSILON_REL) -> list:

    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return []

    approx_contours = []
    for cnt in contours:
        perimeter = cv2.arcLength(cnt, closed=False)
        epsilon   = epsilon_rel * perimeter if perimeter > 0 else 1.0
        approx    = cv2.approxPolyDP(cnt, epsilon, closed=False)
        approx_contours.append(approx)

    def sample_all(step):
        seen, result = set(), []
        for approx in approx_contours:
            pts = approx.reshape(-1, 2).astype(float)
            for i in range(len(pts) - 1):
                for pt in sample_segment(pts[i], pts[i + 1], step):
                    if pt not in seen:
                        seen.add(pt)
                        result.append(pt)
            last = (int(pts[-1][0]), int(pts[-1][1]))
            if last not in seen:
                seen.add(last)
                result.append(last)
        return result

    points       = sample_all(step_px)
    current_step = step_px
    while len(points) < min_points and current_step > 0.5:
        current_step *= 0.7
        points = sample_all(current_step)

    if max_points is not None and len(points) > max_points:
        indices = np.round(np.linspace(0, len(points) - 1, max_points)).astype(int)
        points  = [points[i] for i in indices]

    return points


def nearest_neighbor_sort(points: list) -> list:
    """
    Ordena los puntos con el algoritmo Nearest Neighbor (TSP greedy):
    parte del primer punto y en cada paso va al punto no visitado más cercano.
    Resultado: una sola trayectoria continua que recorre todos los puntos.
    """
    if len(points) == 0:
        return points

    pts  = np.array(points, dtype=float)
    n    = len(pts)
    visited = np.zeros(n, dtype=bool)
    order   = np.empty(n, dtype=int)

    # Punto de inicio: el más cercano a la esquina superior izquierda
    start   = np.argmin(pts[:, 0] + pts[:, 1])
    current = start

    for i in range(n):
        order[i]        = current
        visited[current] = True

        if i < n - 1:
            # Calcular distancias al punto actual para todos los no visitados
            diff  = pts - pts[current]
            dists = diff[:, 0] ** 2 + diff[:, 1] ** 2   # distancia² (evitar sqrt)
            dists[visited] = np.inf                       # ignorar ya visitados
            current = int(np.argmin(dists))

    return [points[i] for i in order]


def export_points(points: list, path: str):
    """Exporta en formato:  x1,y1; x2,y2; x3,y3; ..."""
    content = "; ".join(f"{float(x)},{float(y)}" for x, y in points) + ";"
    Path(path).write_text(content)
    print(f"Puntos guardados en: {path}")


def visualize(img: np.ndarray, edges: np.ndarray, points: list):
    """Muestra original | bordes | trayectoria ordenada como línea continua."""
    # Dibujar la trayectoria como polilínea
    canvas = np.zeros_like(img)
    if len(points) > 1:
        pts_array = np.array(points, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(canvas, [pts_array], isClosed=False, color=(0, 255, 100), thickness=1)
    # Marcar inicio (verde) y fin (rojo)
    cv2.circle(canvas, points[0],  3, (0, 255,   0), -1)
    cv2.circle(canvas, points[-1], 3, (0,   0, 255), -1)

    edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    h, w      = img.shape[:2]
    scale     = min(1.0, 1200 / (w * 3))
    def rs(im): return cv2.resize(im, (int(w * scale), int(h * scale)))

    row = np.hstack([rs(img), rs(edges_bgr), rs(canvas)])
    cv2.imshow('Original  |  Canny  |  Trayectoria ordenada (verde=inicio, rojo=fin)', row)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description='Canny → puntos ordenados para trayectoria continua')
    parser.add_argument('image',                              help='Ruta de la imagen')
    parser.add_argument('--min-points',  type=int,   default=DEFAULT_MIN_POINTS)
    parser.add_argument('--max-points',  type=int,   default=DEFAULT_MAX_POINTS)
    parser.add_argument('--canny-low',   type=int,   default=DEFAULT_CANNY_LOW)
    parser.add_argument('--canny-high',  type=int,   default=DEFAULT_CANNY_HIGH)
    parser.add_argument('--step',        type=float, default=DEFAULT_STEP_PX)
    parser.add_argument('--epsilon-rel', type=float, default=DEFAULT_EPSILON_REL)
    parser.add_argument('--output',      type=str,   default='puntos.txt')
    parser.add_argument('--visualize',   action='store_true')
    args = parser.parse_args()

    img    = load_image(args.image)
    edges  = detect_edges(img, args.canny_low, args.canny_high)
    points = extract_points(edges,
                            min_points  = args.min_points,
                            max_points  = args.max_points,
                            step_px     = args.step,
                            epsilon_rel = args.epsilon_rel)

    print(f"Puntos extraídos: {len(points)}")

    # Ordenar para trayectoria continua
    print("Ordenando puntos (Nearest Neighbor)...")
    points = nearest_neighbor_sort(points)
    print("Ordenamiento completo.")

    print(f"Primeros 5: {'; '.join(f'{float(x)},{float(y)}' for x, y in points[:5])};")

    export_points(points, args.output)

    if args.visualize:
        visualize(img, edges, points)

    return points


if __name__ == '__main__':
    points = main()