#include "../compiler/compiler.c"
int main(){
    char *files[] = {"../examples/alu.cirq","../examples/ram.cirq"};
    for(int i=0;i<2;i++){
        printf("=== TEST %s\n",files[i]);
        Module m = parse_module_file(files[i]); // implement file reader
        run_module(m);
    }
    return 0;
}
