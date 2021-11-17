'''
logger.py
Alex Elias

Logs robot execution to a file
'''

import datetime
import time
import numpy as np

import general_robotics_toolbox as rox

class Logger():
	def __init__(self):
		self.file = None
		self.is_logging = False
		self.t_0  = None

	def log(self, q, egm, cmd, FT, f_des, run_status):
		if not self.is_logging:
			return

		if self.t_0 is None:
			self.t_0 = time.perf_counter()
			ts = 0.0
		else:
			ts = time.perf_counter() - self.t_0

		row = [ts] + list(q) + self.T2row(egm) + self.T2row(cmd) + list(FT) + [f_des, run_status]
		txt = ",".join(str(x) for x in row) + '\n'
		self.file.write(txt)


	def start_logging(self, file_name):
		if self.is_logging:
			return

		datetime_str = datetime.datetime.now()

		self.is_logging = True
		self.t_0 = None
		self.file = open(f"{datetime_str:%Y-%m-%d_%H-%M-%S}_{file_name}", 'x')

		self.file.write(f"# {file_name}\n")
		self.file.write( "# Robotic deep rolling log\n")
		self.file.write(f"# Generated on {datetime_str}\n")
		self.file.write("# \n")
		self.file.write("# Toolpath file:\n") # TODO
		self.file.write("# Comments from toolpath file: \n")

		self.file.write( "# Offsets:\n")
		# TODO

		self.file.write("# Toolpath execution parameters:\n")
		# TODO

		header_cols = []
		header_cols.append("timestamp,")
		header_cols.append("EGMq1,EGMq2,EGMq3,EGMq4,EGMq5,EGMq6,")
		header_cols.append(self.header_T("EGM"))
		header_cols.append(self.header_T("Commanded"))
		header_cols.append("Tx,Ty,Tz,Fx,Fy,Fz,")
		header_cols.append("Fz_des,")
		header_cols.append("run_status")
		header = ''.join(header_cols) + '\n'
		self.file.write(header)

	def header_T(self, x):
		return f"{x}_X,{x}_Y,{x}_Z,{x}_qw,{x}_qx,{x}_qy,{x}_qz,"

	def stop_logging(self):
		if self.is_logging:
			self.is_logging = False
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

	#ts = 1.234
	q = np.array([0.0, 1.1, 2.2, 3.3, 4.4, 5.5])
	egm = rox.random_transform()
	cmd = rox.random_transform()
	FT = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
	f_des = 1000.0
	run_status = 404

	N = 10
	try:
		logger.start_logging("test_log.csv")
		t_0 = time.perf_counter()
		for i in range(N):
			logger.log(q, egm, cmd, FT, f_des, run_status)
		t_1 = time.perf_counter()
	finally:
		logger.stop_logging()
	print((t_1-t_0) / N / 0.004 * 100, '%')
if __name__ == '__main__':
	main()