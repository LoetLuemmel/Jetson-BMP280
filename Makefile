temperature:temperature.c bmp280.c bmp280.h bmp280_defs.h
	gcc -Wall temperature.c bmp280.c -o temperature -I. -lm -li2c
clean:
	rm temperature

