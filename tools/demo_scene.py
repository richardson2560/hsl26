# tools/demo_scene.py
"""Interactive scene visualizer for the Phase-3 kinematic testbed."""

import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT_DIR))
sys.path.insert(0, str(ROOT_DIR / "hsl_core"))

import math
from pathlib import Path
from hsl_core.types import Pose2D
from sim.kinematic.common import CircleTarget, Segment, WorldGeometry
from sim.kinematic.raycaster import FirstHitRaycaster
from sim.kinematic.sensors import LidarSensor
from sim.kinematic.visualizer import DoomConfig, TopViewConfig, render_doom, render_top_view, show_doom, show_top_view


def build_demo_world() -> WorldGeometry:
    """Construye un pasillo con esquinas y un oponente dinámico."""
    walls = (
        Segment((-2.0, -1.0), (6.0, -1.0)),   # Pared sur
        Segment((-2.0, 1.0), (4.0, 1.0)),     # Pared norte
        Segment((4.0, 1.0), (4.0, 3.0)),      # Giro al norte
        Segment((6.0, -1.0), (6.0, 3.0)),     # Giro exterior
        Segment((-2.0, -1.0), (-2.0, 1.0)),   # Cierre oeste
    )
    targets = (
        CircleTarget((2.5, 0.0), 0.20, "explorer_rival"),  # Oponente frente a nosotros
    )
    return WorldGeometry(walls, targets)


def main():
    world = build_demo_world()
    
    # 1. Colocar el robot en el pasillo mirando hacia el este (+X)
    robot_pose = Pose2D(0.0, 0.0, 0.0)

    # 2. Configurar el sensor con 360 rayos y el sector ciego de las barras
    sensor = LidarSensor(
        raycaster=FirstHitRaycaster(max_range_m=10.0),
        beam_count=360,
        max_range_m=10.0,
        noise_std_m=0.01,
        seed=20260925,
        blind_sectors=((math.pi / 4.0, 0.08), (-math.pi / 4.0, 0.08)), # Sombras de las barras
    )

    # 3. Tomar una observación
    observation = sensor.observe(robot_pose, stamp_s=1.0, geometry=world)

    print(f"Observación completada: {len(observation.ranges_m)} rayos.")
    print("Guardando imágenes de diagnóstico en 'artifacts/reports/phase3/'...")

    # 4. Renderizar ambas vistas a matrices de imagen
    img_top = render_top_view(robot_pose, world, observation, TopViewConfig(width_px=800, height_px=600))
    img_doom = render_doom(observation, DoomConfig(width_px=640, height_px=360))

    # Guardar como PNG si tienes Matplotlib instalado, o mostrarlas en pantalla
    try:
        import matplotlib.pyplot as plt
        
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        axes[0].imshow(img_top)
        axes[0].set_title("Top-View 2D (Verde: Rayos libres, Rojo: Ciegos)")
        axes[0].axis("off")

        axes[1].imshow(img_doom)
        axes[1].set_title("Doom 2.5D (Perspectiva Frontal del Robot)")
        axes[1].axis("off")

        plt.tight_layout()
        output_path = Path("artifacts/reports/phase3/demo_scene.png")
        output_path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(output_path)
        print(f"¡Imagen combinada guardada en {output_path}!")
        print("Abriendo ventana gráfica...")
        plt.show()
    except ImportError:
        print("Matplotlib no está instalado en este entorno. Las imágenes fueron generadas como arrays NumPy con éxito.")


if __name__ == "__main__":
    main()