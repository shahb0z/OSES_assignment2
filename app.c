/*
 * This is a user-space application that reads /dev/sample
 * and prints the read characters to stdout
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>




int main(int argc, char **argv)
{
	char *app_name = argv[0];
	char *dev_name = "/dev/dist_me";
	int fd = -1;
	int x,blink_rate,iter=0;
	char distance[10];
	float rd;
	long long int some;
	if( argc < 2 )
	{
		printf( "Usage: %s #of measures (0 for infinite)\n", argv[0] );
		return 1;
	}
	sscanf( argv[1], "%d", &iter );
	/*
 	 * Open the sample device RD | WR
 	 */
	if ((fd = open(dev_name, O_RDWR)) < 0) {
		fprintf(stderr, "%s: unable to open %s: %s\n", 
			app_name, dev_name, strerror(errno));
		return( 1 );
	}
	int i;
	if(iter==0)
		i=1;
	else
		i=iter;
	while(i){
		x = read(fd,&distance,sizeof(distance));
		if(x<0){
			printf("OVERFLOW!!!\n");
			return -1;
		}else{
			sscanf(distance,"%lld",&some);
			rd = (float)some/58;
			printf("Distance: %.1f\n",rd);
			if (rd > 100)
				blink_rate=2000;
			else if(rd >=10 )
				blink_rate=((int)rd/25+2)*200;
			else 
				blink_rate=0;
			printf("Blink rate: %d\n",blink_rate);
			if(write(fd, &blink_rate, sizeof( blink_rate) )<0){
				printf("Error while writing blink rate!");
			}
		}
		if(iter!=0)
			i--;
		usleep(100);
	}
	if (fd >= 0) {
		close(fd);
	}
	return( 0 );
}
