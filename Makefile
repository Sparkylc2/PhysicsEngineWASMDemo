EMCC = emcc
SRC = physics_engine.cpp physics_bindings.cpp globals.cpp rigidbody.cpp interaction_bridge.cpp 
OUT = ../../../src/wasm/physics_engine.js 

all:
	$(EMCC) $(SRC) -I. -O2 \
		--bind -sASSERTIONS -s MODULARIZE=1 -s EXPORT_ES6=1 -s ENVIRONMENT=web -msimd128 \
		-s ALLOW_MEMORY_GROWTH=1 \
		-o $(OUT)


clean:
	rm -f ../../../src/wasm/physics_engine.*
	rm -f ../../../src/wasm/*.d.ts
