CC := g++

srcs := $(shell find src -name *.c)
objs := $(patsubst src/%.c,objs/%.o,$(srcs))


include_paths := /interface 
abs_paths := $(shell pwd)
include_paths := $(include_paths:%=$(abs_paths)%)
I_flag := $(include_paths:%=-I%)

compile_optinons := -g -O3 -w $(I_flag)

objs/%.o: src/%.c
	@mkdir -p $(dir $@)
	$(CC) -c $^ -o $@ $(compile_optinons)

workspace/exec : $(objs)
	@mkdir -p $(dir $@) 
	$(CC) $^ -o $@

run:workspace/exec	
	./$<

clean:
	rm -rf objs workspace/exec

debug:
	@echo $(objs)
	@echo $(srcs)
	@echo $(compile_optinons)

.PHONY : run,clean,debug
