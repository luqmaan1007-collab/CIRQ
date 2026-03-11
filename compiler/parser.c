#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    char name[64];
    char body[1024][128];
    int body_lines;
} Module;

Module parse_module(const char *src){
    Module m;
    memset(&m,0,sizeof(Module));
    const char *p = strstr(src,"module ");
    if(!p){ printf("Error: no module\n"); exit(1);}
    sscanf(p,"module %63s",m.name);

    const char *bstart = strchr(src,'{');
    const char *bend = strrchr(src,'}');
    int line_idx = 0;
    char buf[128];
    const char *cur = bstart+1;
    while(cur < bend){
        int n = 0;
        while(*cur!='\n' && cur<bend) buf[n++] = *cur++;
        buf[n]=0;
        strcpy(m.body[line_idx++],buf);
        if(*cur=='\n') cur++;
    }
    m.body_lines = line_idx;
    return m;
}
