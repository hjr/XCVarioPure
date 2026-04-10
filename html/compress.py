import gzip
import shutil
import sys
import os

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: compress.py <input_file> <output_file>")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]

    os.makedirs(os.path.dirname(output_file), exist_ok=True)

    file_name = os.path.basename(input_file)

    print(f"Compressing: {file_name} > {os.path.basename(output_file)}")

    try:
        with open(input_file, 'rb') as f_in:
            with gzip.open(output_file, 'wb', compresslevel=9) as f_out:
                shutil.copyfileobj(f_in, f_out)
    except Exception as e:
        print(f"Error compressing {file_name}: {e}")
        sys.exit(1)