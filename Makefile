decoder: v4l2_decode.c
	gcc -O2 -Wall v4l2_decode.c -o decoder

test.h264:
	ffmpeg -f lavfi -i testsrc=size=1920x1080:rate=30 \
		-frames:v 100 \
		-c:v libx264 \
		-profile:v baseline \
		-level 3.1 \
		-pix_fmt yuv420p \
		-x264-params keyint=30:scenecut=0 \
		-f h264 \
		test.h264
