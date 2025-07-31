execnamebase = main
files = main.c

libflags = -lraylib -lm
debugflags = $(libflags) -Wall -Wextra
releaseflags = $(debugflags) -Werror

release: $(files)
	gcc $(releaseflags) -O3 -o $(execnamebase)-release $(files)

debug: $(files)
	gcc -g $(debugflags) -O0 -o $(execnamebase)-debug $(files)