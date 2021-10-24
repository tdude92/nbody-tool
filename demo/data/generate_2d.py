import sys
import random
import math

# Utility
def number_compact_str(num):
    # Converts a number to a compact string (eg. 10000 -> 10k)
    if num % 1000000000 == 0:
        return str(num//1000000000) + "B"
    elif num % 1000000 == 0:
        return str(num//1000000) + "M"
    elif num % 1000 == 0:
        return str(num//1000) + "k"
    else:
        return str(num)


# Insufficient args
# python3 generate_2d.py [number of bodies] [simulation type (default: stars_random)]
if len(sys.argv) < 2:
    print("Usage: python3 generate_2d.py [number of bodies] [simulation type (default: stars_uniform)]")
    print()
    print("Simulation Types:")
    print("-------------------")
    print("1. stars_uniform")
    print("2. solarsystem_uniform")
    print()
    quit()

# Initialize params
N_BODIES = int(sys.argv[1])

if len(sys.argv) < 3:
    SIM_TYPE = "stars_uniform" # default
else:
    SIM_TYPE = sys.argv[2]

FILE_NAME  = "2d/" + SIM_TYPE + "_"
FILE_NAME += number_compact_str(N_BODIES) + ".data"

# Generate data based on distribution type
if SIM_TYPE == "stars_uniform":
    # Length: Light year
    # Mass: Solar mass
    # Time: Millenium
    min_m = 0.1
    max_m = 120
    min_x = 0
    max_x = 10000
    min_v = 0
    max_v = 5
    with open(FILE_NAME, "w") as wf:
        for _ in range(N_BODIES):
            m = [random.uniform(min_m, max_m)]
            r = [0] # Radius insignificant
            x = [random.uniform(min_x, max_x) for _ in range(2)]
            v = [random.uniform(min_v, max_v) for _ in range(2)]
            wf.write(" ".join([str(i) for i in m + r + x + v]) + "\n")
elif SIM_TYPE == "solarsystem_uniform":
    # Length: Astronomical unit
    # Mass: Earth mass
    # Time: Earth day
    min_m = 0.001
    max_m = 200
    min_density = 4e+11
    max_density = 3e+12
    min_x = 0
    max_x = 80
    min_v = 0
    max_v = 0.05
    with open(FILE_NAME, "w") as wf:
        # Central star (just using the sun's physical data)
        density = 7.9043e+11
        m = [333000]
        r = [0.00465047]
        x = [(min_x + max_x)/2]*2
        v = [0, 0]
        wf.write(" ".join([str(i) for i in m + r + x + v]) + "\n")

        # Other bodies
        for _ in range(N_BODIES - 1):
            density = random.uniform(min_density, max_density)
            m = [random.uniform(min_m, max_m)]
            r = [((3*m[0])/(4*math.pi*density))**(1/3)]
            x = [random.uniform(min_x, max_x) for _ in range(2)]
            v = [random.uniform(min_v, max_v) for _ in range(2)]
            wf.write(" ".join([str(i) for i in m + r + x + v]) + "\n")