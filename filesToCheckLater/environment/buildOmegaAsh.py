import subprocess
import select
import sys
import os


def buildOmegaAsh():
    import pty

    pty, tty = pty.openpty()

    p = subprocess.Popen(['docker', 'exec', '-i', '-t',
                          'omega-ash', 'bash'], stdin=tty, stdout=tty, stderr=tty)

    os.write(pty, 'cd omega-ash && mkdir build\n'.encode())
    os.write(pty, 'cd build\n'.encode())
    os.write(pty, 'cmake ../\n'.encode())
    os.write(pty, 'make\n'.encode())
    os.write(pty, './hello\n'.encode())

    while p.poll() is None:
        r, _, _ = select.select([sys.stdin, pty], [], [])
        if sys.stdin in r:
            input_from_your_terminal = os.read(sys.stdin.fileno(), 10240)
            os.write(pty, input_from_your_terminal)
        elif pty in r:
            output_from_docker = os.read(pty, 10240)
            os.write(sys.stdout.fileno(), output_from_docker)

# bashCommand = "docker exec -i -t omega-ash bash"
# subprocess.Popen(bashCommand, shell=True)

# process = subprocess.Popen(
#     bashCommand, stdout=subprocess.PIPE, shell=True, universal_newlines=True)

# output, error = process.communicate()

# print('out: {0}'.format(output))
