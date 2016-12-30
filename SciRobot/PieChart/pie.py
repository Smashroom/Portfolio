#!/usr/bin/env python
import matplotlib.pyplot as pyplot

x_list=[1,3,6]
label_list=["bells","whistles","pasta"]

pyplot.axis("equal")
pyplot.pie(
	x_list,
	labels=label_list,
	autopct="%1.1f%%"
)

pyplot.title("Pastafarianism expenses")
pyplot.show()
