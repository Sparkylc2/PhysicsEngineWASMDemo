EMCC = emccAdd commentMore actions
SRC = physics_engine.cpp
OUT = ../../../src/wasm/physics_engine.js
EIGEN = ../../../lib/cpp

all:
	$(EMCC) $(SRC) -I$(EIGEN) -O2 \
		--bind -sASSERTIONS -s MODULARIZE=1 -s EXPORT_ES6=1 -s ENVIRONMENT=web -msimd128 \
		-o $(OUT)

clean:
	rm -f ../../../src/wasm/physics_engine.*
	rm -f ../../../src/wasm/*.d.ts
