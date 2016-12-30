import matplotlib.pyplot as pyplot

def eatthePie(timeData,nameData):
	pyplot.axis('equal')  #To print a pie chart
	pyplot.pie(
		timeData,
		labels=nameData,
		autopct="%1.1f%%"
	) 

	pyplot.title('Activities Pie Chart :D')
	pyplot.show()
