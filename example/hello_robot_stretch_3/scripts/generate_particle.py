# generate_particles_in_cup.py
import random
import os
import math

def generate_particles_in_cup_xml(filename="cup_particles.xml",
                                   num_particles=10,
                                   cup_center=(0.5, -0.55, 0.61),
                                   cup_radius=0.026,
                                   cup_height=0.07,
                                   particle_radius=0.005):
    """
    Generate a MuJoCo XML file with free-floating balls inside a hollow cup.

    :param filename: output XML file (saved in script's directory)
    :param num_particles: number of particles
    :param cup_center: (x, y, z) center of cup base
    :param cup_radius: inner radius of cup
    :param cup_height: height of cup
    :param particle_radius: radius of particles
    """
    # Script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filepath = os.path.join(script_dir, filename)

    with open(filepath, "w") as f:
        f.write('<mujoco model="particles">\n')

        for i in range(num_particles):
            # Random angle and distance within cup radius, keep particle inside
            r = random.uniform(0, cup_radius - particle_radius)
            theta = random.uniform(0, 2 * math.pi)
            x_offset = r * math.cos(theta)
            y_offset = r * math.sin(theta)
            z = cup_center[2] + particle_radius + random.uniform(0, cup_height - 2*particle_radius)

            color = [random.random(), random.random(), random.random()]
            color_str = f"{color[0]:.3f} {color[1]:.3f} {color[2]:.3f} 1"

            f.write(f'  <body name="particle{i}" pos="{cup_center[0]+x_offset:.5f} {cup_center[1]+y_offset:.5f} {z:.5f}">\n')
            f.write('    <freejoint/>\n')
            f.write('    <inertial pos="0 0 0" mass="1e-4" diaginertia="1e-8 1e-8 1e-8"/>\n')
            f.write(f'    <geom type="sphere" size="{particle_radius}" rgba="{color_str}"/>\n')
            f.write('  </body>\n')

        f.write('</mujoco>\n')

    print(f"Particles inside cup XML generated as '{filepath}'")

if __name__ == "__main__":
    # Example: generate 10 particles
    generate_particles_in_cup_xml("particles_free.xml", num_particles=10)
