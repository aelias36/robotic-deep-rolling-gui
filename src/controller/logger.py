'''
logger.py
Alex Elias

Logs robot execution to a file
'''

import datetime
import numpy as np

import general_robotics_toolbox as rox

class Logger():
	def __init__(self):
		self.file = None

	def log(self, ts, q, egm, cmd, FT, f_cmd, run_status):
		row = [ts] + list(q) + self.T2row(egm) + self.T2row(cmd) + list(FT) + [f_cmd, run_status]
		txt = ",".join(str(x) for x in row) + '\n'
		self.file.write(txt)


	def start_logging(self, file_name):
		self.file = open(file_name, 'x')

		self.file.write(f"# {file_name}\n")
		self.file.write( "# Robotic deep rolling log\n")
		self.file.write(f"# Generated on {datetime.datetime.now()}\n")
		self.file.write("# \n")
		self.file.write("# Toolpath file:\n") # TODO
		self.file.write("# Comments from toolpath file")

		self.file.write( "# Offsets:\n")
		# TODO

		self.file.write("# Toolpath execution parameters:\n")
		# TODO

		header_cols = []
		header_cols.append("timestamp,")
		header_cols.append("EGM q1,EGM q2,EGM q3,EGM q4,EGM q5,EGM q6,")
		header_cols.append(self.header_T("EGM"))
		header_cols.append(self.header_T("Commanded"))
		header_cols.append("Fx,Fy,Fz,Tx,Ty,Tz,")
		header_cols.append("FzCMD,")
		header_cols.append("run status")
		header = ''.join(header_cols) + '\n'
		self.file.write(header)

	def header_T(self, x):
		return f"{x} X,{x} Y,{x} Z,{x} qw,{x} qx,{x} qy,{x} qz,"

	def stop_logging(self):
		self.file.write("# End of log")
		self.file.close()


	def T2row(self, T):
		q = rox.R2q(T.R)
		return list(T.p)+list(q)

def main():
	import time

	logger = Logger()
	# T = rox.random_transform()
	# print(logger.T2row(T))

	ts = 1.234
	q = np.array([0.0, 1.1, 2.2, 3.3, 4.4, 5.5])
	egm = rox.random_transform()
	cmd = rox.random_transform()
	FT = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
	f_cmd = 1000.0
	run_status = 404

	N = 10
	try:
		logger.start_logging("test_log.csv")
		t_0 = time.perf_counter()
		for i in range(N):
			logger.log(ts, q, egm, cmd, FT, f_cmd, run_status)
		t_1 = time.perf_counter()
	finally:
		logger.stop_logging()
	print((t_1-t_0) / N / 0.004 * 100, '%')
if __name__ == '__main__':
	main()