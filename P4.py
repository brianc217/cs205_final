import numpy as np
import matplotlib.pyplot as plt
from mpi4py import MPI

from P4_serial import *

def slave(comm):
  status = MPI.Status()
  data = np.zeros(2, dtype=np.float)

  while True:
    comm.recv(data, source=0)
    rowNum = data[0]
    yVal = data[1]

    row = []
    for j,x in enumerate(np.linspace(minX, maxX, width)):
      row.append(mandelbrot(x, yVal))

    comm.send(row, tag=rowNum, dest=0)

  return

def master(comm):
  image = np.zeros([height,width], dtype=np.uint16)
  rows = []

  for i,y in enumerate(np.linspace(minY, maxY, height)):
    #image[i,j] = mandelbrot(x,y)
    rows.append([float(i), y])

  #send first round to all slaves
  for i in xrange(size):
    if i == 0:
      continue
    print rows[0]
    comm.send(rows.pop(0), dest=i)

  data = np.zeros(width, dtype=np.uint16)
  while len(rows) > 0:
    status = MPI.Status()
    comm.recv(data, source=MPI.ANY_SOURCE, tag=MPI.ANY_TAG, status=status)
    proc = status.Get_source()
    rowNum = status.Get_tag()
    image[rowNum, :] = data
    comm.send(rows.pop(0), dest=proc)


  return image


if __name__ == '__main__':
  # Get MPI data
  comm = MPI.COMM_WORLD
  rank = comm.Get_rank()
  size = comm.Get_size()

  if rank == 0:
    start_time = MPI.Wtime()
    C = master(comm)
    end_time = MPI.Wtime()
    print "Time: %f secs" % (end_time - start_time)
    plt.imsave('Mandelbrot.png', C, cmap='spectral')
    plt.imshow(C, aspect='equal', cmap='spectral')
    plt.show()
  else:
    slave(comm)
