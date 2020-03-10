# -*- coding: utf-8 -*-
"""
Created on Wed Nov 7 18:54:21 2018

@author: Andy Everitt (aje2g15@soton.ac.uk)
"""

import numpy as np
import cv2


class meta_data:
	def __init__(self, name):
		self.name = name

	def get_meta(self, data):
		self.ncols = int(data[0].split(' ')[-1])
		self.nrows = int(data[1].split(' ')[-1])
		self.xllcenter = float(data[2].split(' ')[-1])
		self.yllcenter = float(data[3].split(' ')[-1])
		self.xcellsize = float(data[4].split(' ')[-1])
		self.ycellsize = float(data[5].split(' ')[-1])
		self.NODATA_value = float(data[6].split(' ')[-1])


class map_data:
	def __init__(self, name, data, meta):
		self.name = name
		self.data = data
		self.meta = meta


def get_data(filename, filepath):
	fr = open("{}/{}".format(filepath, filename))
	data = fr.read().split('\n')
	fr.close()
	return(data)


def create_map(raw_data, filename="blank"):
	i = 0
	prev_percent = 0

	meta = meta_data(filename)
	meta.get_meta(raw_data)							# retrieve meta data
	raw_data = raw_data[7:]							# remove meta data from list
	data = np.zeros([meta.nrows, meta.ncols])		# create empty array
	for row in range(meta.nrows):
		row_data = raw_data[row].split(' ')		# split row into individual strings
		for col in range(meta.ncols):
			if (row_data[col] != ('')):
				print("row = {}: col = {}".format(row, col))
				data[row, col] = float(row_data[col])	# update array

				i += 1
				percent = int((i / (meta.nrows*meta.ncols)) * 100)
				if (percent % 10 == 0 and percent != prev_percent):
					prev_percent = percent
					print(percent, '%')

	data[data==meta.NODATA_value] = data[data!=meta.NODATA_value].mean()

	new_map = map_data(filename, data, meta)
	return(new_map)


if __name__ == "__main__":
	filepath = "./Utah/USGS"
	filename = "n39w111_10m_129.asc"

	raw_data = get_data(filename, filepath)
	raw_map = create_map(raw_data, filename)

	np.save(filename.rstrip(".asc"), raw_map.data)

	a = 1