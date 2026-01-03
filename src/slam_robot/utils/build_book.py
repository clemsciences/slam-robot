import os
import pathlib
import subprocess
import sys

from slam_robot import PACKDIR


def main():
    # Get the project root directory
    # Assuming this script is in src/slam_robot/utils/build_book.py
    # Root is three levels up from this file
    # But when installed as a package, we should probably find the root differently.
    # Alternatively, we can assume it's run from the project root.


    pack_dir = pathlib.Path(PACKDIR)
    book_path = os.path.join(pack_dir.parent, "course")
    
    if not os.path.exists(book_path):
        print(f"Error: Could not find book path at {book_path}")
        sys.exit(1)
        
    print(f"Building jupyter-book at {book_path}...")
    try:
        subprocess.run(["jupyter-book", "build", "src/course"], check=True)
        print("Book built successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Error building book: {e}")
        sys.exit(e.returncode)
    except FileNotFoundError:
        print("Error: jupyter-book command not found. Please install jupyter-book.")
        sys.exit(1)

if __name__ == "__main__":
    main()
