import subprocess

# Start the C++ program as a subprocess
process = subprocess.Popen(["NewVersion/Algorithm/AutoFlow"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)

# Send data to the C++ program
input_data = "Hello from Python"
process.stdin.write(input_data)
process.stdin.flush()

# Read the output from the C++ program
output_data = process.stdout.readline()
print(f"C++ says: {output_data}")

process.stdin.close()
process.wait()
