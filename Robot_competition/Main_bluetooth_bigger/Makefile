all:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c fsensor.c -o obj/fsensor.o
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c fmotor.c -o obj/fmotor.o
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c main.c -o obj/main.o
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c actions.c -o obj/actions.o
	gcc obj/fsensor.o obj/fmotor.o obj/main.o obj/actions.o -Wall -lm -lev3dev-c -lbluetooth -lpthread -o obj/main
actions:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c actions.c -o obj/actions.o
	gcc obj/fsensor.o obj/fmotor.o obj/main.o obj/actions.o -Wall -lm -lev3dev-c -lbluetooth -lpthread -o obj/main
main:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c mainBlu.c -o obj/mainBlu.o
	gcc obj/fsensor.o obj/fmotor.o obj/main.o obj/actions.o -Wall -lm -lev3dev-c -lbluetooth -lpthread -o obj/main
fmotor:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c fmotor.c -o obj/fmotor.o
	gcc obj/fsensor.o obj/fmotor.o obj/main.o obj/actions.o -Wall -lm -lev3dev-c -lbluetooth -lpthread -o obj/main
fsensor:
	gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment -c fsensor.c -o obj/fsensor.o
	gcc obj/fsensor.o obj/fmotor.o obj/main.o obj/actions.o -Wall -lm -lev3dev-c -lbluetooth -lpthread -o obj/main
run:
	obj/main
reset:
	echo reset > /sys/class/tacho-motor/motor0/command
	echo reset > /sys/class/tacho-motor/motor1/command
	echo reset > /sys/class/tacho-motor/motor2/command
