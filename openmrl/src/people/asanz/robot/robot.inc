define miniskybot_sonar ranger(

	scount 4
	spose[0] [-0.067 0.022 162]
	spose[1] [0 0.0715 90]
	spose[2] [0.067 0.022 18]
	spose[3] [0 -0.0715 -90]
)

define miniskybot_irl ranger(

	scount 5
	spose[0] [-0.076 0.0255 162]
	spose[1] [0 0.0805 90]
	spose[2] [0.076 0.0255 18]
	spose[3] [0.047 -0.065 -54]
	spose[4] [0.047 -0.065 -126]
)

define miniskybot_irs ranger(

	scount 2
	spose[0] [-0.047 0.065 126]
	spose[1] [0.047 0.065 54]
)


define miniskybot position(

	size [0.150 0.143 0.01]
	origin [0.071 0.0745 0.05 0]

	block(
		points 10
		point[0] [0.052 0]
		point[1] [0.097 0]
		point[2] [0.135 0.0275]
		point[3] [0.150 0.071]
		point[4] [0.135 0.115]
		point[5] [0.097 0.143]
		point[6] [0.052 0.143]
		point[7] [0.0145 0.115]
		point[8] [0 0.071]
		point[9] [0.0145 0.0275]
		z [0 0.01]
	)

	drive "diff"

	miniskybot_sonar()
	miniskybot_irl()
	miniskybot_irs()
)


		
