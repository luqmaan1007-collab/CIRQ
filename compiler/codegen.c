#include "parser.c"
#include <stdio.h>

void run_module(Module m){
    printf("Running module %s\n", m.name);
    for(int i=0;i<m.body_lines;i++){
        printf("EXEC> %s\n", m.body[i]); // Replace with virtual hardware logic
    }
}
