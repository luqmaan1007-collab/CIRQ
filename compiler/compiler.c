#include "parser.c"
#include "codegen.c"
#include <stdio.h>
#include <stdlib.h>

int main(int argc,char **argv){
    if(argc<2){ printf("Usage: ./compiler <file.cirq>\n"); return 1; }
    FILE *f = fopen(argv[1],"r");
    if(!f){ perror("File open"); return 1;}
    fseek(f,0,SEEK_END);
    long sz = ftell(f); fseek(f,0,SEEK_SET);
    char *src = malloc(sz+1); fread(src,1,sz,f); src[sz]=0; fclose(f);
    Module m = parse_module(src);
    run_module(m);
    free(src);
    return 0;
}
