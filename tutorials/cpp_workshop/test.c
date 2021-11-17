#include <stdio.h>

int main(){
	
	int i;
	
	scanf("%d", &i);
	
	int arr[i];
	for(int n = 0; n < i; n++){
	    arr[n] = n;
	}

	for(int n = 0; n < i; n++){
	    printf("%d", arr[n]);
	}

}
