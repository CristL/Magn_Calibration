9150_test:9150_test.o fit_ellipse.o
	arm-arago-linux-gnueabi-gcc -lm -o 9150_test 9150_test.o fit_ellipse.o
9150_test.o:9150_test.c
	arm-arago-linux-gnueabi-gcc -std=gnu99 -lm -c 9150_test.c
fit_ellipse.o:fit_ellipse.c
	arm-arago-linux-gnueabi-gcc -std=gnu99 -lm -c fit_ellipse.c
	
clean:
	@rm *.o 9150_test