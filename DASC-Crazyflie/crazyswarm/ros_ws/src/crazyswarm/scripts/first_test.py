from pycrazyswarm import *
from hover_and_land import *


# Tesing take off, hover, and land 
def main():
    swarm = Crazyswarm()
    print("Testing taking off and hovering...")
    hover_all(swarm)
    print("Testing landing...")
    land_all(swarm)

if __name__ == "__main__":
    main()
