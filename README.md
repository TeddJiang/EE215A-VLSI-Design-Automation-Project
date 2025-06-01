## Make the Project
```bash
   mkdir build
   cd build
   cmake ..
   make
```

## Runing the router with A* alogrithm and Preferred Direction Routing
We use A* algorithm to further accelerate the routing algorithm, using -a to enable A* algorithm.

Preferred Direction Routing is enabled by -s, -s 0 means that we are doning preferred direction routing.

## Command to run the benchmarks
You should enter the /bin directory to run the route.
1. Using the Preferred Direction Routing with/without A*
```bash
    ./route -f bench1 -a 1
    ./route -f bench2 -a 1
    ./route -f bench3 -a 1
    ./route -f bench4 -a 1
    ./route -f bench5 -a 1
    ./route -f fract2 -a 1
    ./route -f industry1 -a 1
    ./route -f primary1 -a 1
    ./route -f bench1
    ./route -f bench2
    ./route -f bench3
    ./route -f bench4
    ./route -f bench5
    ./route -f fract2
    ./route -f industry1
    ./route -f primary1
```

1. Do not use the Preferred Direction Routing with/without A*
```bash
    ./route -f bench1 -a 1 -s 1
    ./route -f bench2 -a 1 -s 1
    ./route -f bench3 -a 1 -s 1
    ./route -f bench4 -a 1 -s 1
    ./route -f bench5 -a 1 -s 1
    ./route -f fract2 -a 1 -s 1
    ./route -f industry1 -a 1 -s 1
    ./route -f primary1 -a 1 -s 1
    ./route -f bench1 -s 1
    ./route -f bench2 -s 1
    ./route -f bench3 -s 1
    ./route -f bench4 -s 1
    ./route -f bench5 -s 1
    ./route -f fract2 -s 1
    ./route -f industry1 -s 1
    ./route -f primary1 -s 1
```