﻿# pixel2points
Given an input directory of >3-band tifs, returns the spatially referenced coordinates of each black (values less than 20,20,20) pixel.
Saves the results either to a csv file or a shapefile consisting of multipoint features (one for each image).
