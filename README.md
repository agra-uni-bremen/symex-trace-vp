# SymEx-Trace-VP

A tracing extension for the 
[SymEx-Vp][SymEx-VP] [concolic testing][wikipedia ct] framework for RISC-V embedded software with support for [SystemC][systemc website] peripherals.

## About

SymEx-Trace-VP is an extension of the existing [SymEx-VP][SymEx-VP] 
that collects information about each executed instruction during 
symbolic execution and outputs it in an XML based format. 
This trace file is designed to be used as input for the symbolic execution 
visualization tool [SymEx-3D][SymEx-3D]. 

SymEx-VP focuses explicitly on testing software for constrained embedded
devices (e.g. as used in the Internet of Things). This software often
interacts very closely with low-level hardware peripherals and in order
to support these interactions, during simulation-based software testing,
SymEx-VP supports [SystemC][systemc website] peripheral models. SystemC
is a C++ class library for modeling hardware peripherals. SystemC is
often used to create executable models of an entire hardware platform,
so-called Virtual Prototypes (VPs). Using these SystemC peripherals
models, SymEx-VP allows injecting test inputs into the software
simulation via the [MMIO][wikipedia mmio] peripheral interface. Thereby
allowing tests of embedded software with only minimal software
modifications.

Based on injected inputs, SymEx-VP allows [symbolic execution][wikipedia
symex] (or more specifically [concolic testing][wikipedia ct]) of RISC-V
RV32IMAC machine code. Branches based on injected symbolic values are
tracked and as soon as execution terminates new assignments for symbolic
variables are determined by negating encountered branch conditions
(Dynamic Symbolic Execution). For each new assignment, the software
[simulation is restarted][systemc restart] from the beginning, thereby
(ideally) enabling exploration of all paths through the program based on
the introduced symbolic variables.

All instructions for installing, building and running the SymEx-Trace-VP are identical 
to those of the original [SymEx-VP][SymEx-VP]. 

Details about the symbolic execution visualization are 
described in [this][SymEx-3D] repository and 
in the [3D SymEx Visualization][3D SymEx Visualization] paper. 
More details on SymEx-VP are provided in the
[SymEx-VP paper][symex-vp paper]

## Features

* Concolic execution of RISC-V RV32IMAC machine code
* Accurate hardware peripheral modeling via the SystemC C++ library
* Support for injecting concolic test inputs via the MMIO peripheral interface
* Support for generating test case files and replaying them (see `SYMEX_TESTCASE` below)
* Integrated [GDB][gdb website] stub to ease debugging of encountered errors (`--debug-mode`)
* Support for many embedded operating systems (e.g. [RIOT][riot website], [Zephyr][zephyr website], …)
* **Symbolic execution trace generation, ready for use with [SymEx-3D][SymEx-3D]**

## Cloning

This repository makes use of submodules to include vendored dependencies.
In order to automatically checkout these submodules clone the repository
as follows:

	$ git clone --recursive https://github.com/agra-uni-bremen/symex-trace-vp

Alternatively, if you already cloned the repository without passing the
`--recursive` option run the following command to checkout all submodules:

	$ git submodule update --init

## Installation

This software can be installed either using [Docker][docker website]
(recommended) or manually via [GNU make][make website].

### Docker

To build a Docker image for SymEx-VP run the following command:

	$ docker build -t riscv-symex .

Afterwards, create a new Docker container from this image using:

	$ docker run --rm -it riscv-symex

The SymEx-VP source directory is then available in
`/home/riscv-vp/riscv-vp` within the container. Provided VPs are
automatically added to `$PATH`. For this reason, the examples provided
in `/home/riscv-vp/riscv-vp/examples` (see below) can easily be executed
within the provided container.

### Manual

Manual installation requires the following software to be installed:

* A C++ compiler toolchain with C++17 support
* [CMake][cmake website]
* A recent version of [Z3][z3 repo] (`4.8.X` is known to work)
* [LLVM][llvm website]
* [Boost][boost website]

After all dependencies have been installed run:

	$ make

Executable binaries are then available in `./vp/build/bin`.

## Usage

In regards to using SymEx-VP for software execution it behaves like a
normal virtual prototype and should be able to execute any RV32IMAC
binaries. In order to utilize the symbolic execution features provided
by SymEx-VP, the following additional aspects have to be consider for
software testing. Communication between software and the virtual
prototype is achieved through memory-mapped IO with a provided
`SymbolicCTRL` peripheral.

1. **Symbolic Inputs:** In order to explore different paths through the
   program, inputs based on which path exploration should take place
   need to be marked as symbolic. Symbolic variables can be introduced
   through SystemC peripherals which return symbolic variables through
   a TLM 2.0 extension as part of memory-mapped I/O. For example, it is
   possible to model an Ethernet peripheral using SystemC which returns
   symbolic variables for network packets and thereby allows exploring
   paths through the network stack of the executed software.
   Alternatively, it is also possible to declare variables as symbolic
   manually through the aforementioned `SymbolicCTRL` peripheral.
2. **Termination Points:** Since SymEx-VP restarts the entire SystemC
   simulation for each new assignment of symbolic input variables,
   termination points need to be defined for the simulated software. For
   example, when exploring the network stack of the executed software,
   it may be sufficient to terminate software simulation as soon as the
   symbolic network packet was handled by the network stack. Termination
   points must presently be declared by the executed software by writing
   to a control register in the memory-mapped `SymbolicCTRL` peripheral.
3. **Path Analyzers:** In order to find errors using symbolic execution,
   a so-called path analyzer needs to be employed on each executed path.
   For each executed path, the employed path analyzer needs to decide if
   the path constitutes an error case. For example, errors may include
   violation of [spatial memory safety][dac checkedc],
   [stack overflows][fdl stack], et cetera. Presently, we mostly rely on
   the executed software to signal an error condition to the virtual
   prototype using the `SymbolicCTRL` peripheral. For example, this
   allows signaling errors from software-specific panic handlers.

### Trace generation
The SymEx-VP automatically collects information about each executed instruction for 
all discovered paths. 
The resulting trace is written to the standard output stream in a human readable format. 

If the symbolic execution is aborted either manually or after an error, 
it still prints the partial trace for all previous discovered paths. 

This trace file is designed to be used as input for the symbolic execution 
visualization tool [SymEx-3D][SymEx-3D]. 
The visualization tool currently supports DWARF debug info version 4. 



## Usage Examples

Examples can be found in the `./examples` subdirectory. 
The example `./examples/trace-example` contains several different program constructs
(like recursion and branches) that are well suited to demonstrate  
different aspects of symbolic program execution. 

Executing the example with the built `symex-vp` using

	$ ./symex-vp ./examples/trace-example/main

or the `sim` target in the Makefile results in [this][trace example] trace file. 

For additional examples on how to run programs on the SymEx-VP 
see the usage section in the original [SymEx-VP][SymEx-VP] repository. 





## Provided VPs

The following virtual prototypes are available:

* `symex-vp`: A very minimal virtual prototype, based on `tiny32-vp`
  from the original riscv-vp repository. This allows testing very basic
  bare-metal RV32 software.
* `hifive-vp`: A virtual prototype mostly compatible with the
  [SiFive HiFive1][sifive hifive1]. This allows executing software
  for embedded operating systems like [RIOT][riot website] or
  [Zephyr][zephyr website] symbolically.
* `test32-vp`: This virtual prototype is intended to be used with
  the [riscv-compliance][riscv-compliance github] repository. This is
  primarily useful for development (e.g. during testing of new
  RISC-V extensions).

## Environment Variables

The following environment variables can be set:

* **SYMEX_ERREXIT:** If set, terminate after encountering the first
  error during symbolic execution of the software.
* **SYMEX_TIMEOUT:** This can be used to configure the solver timeout
  of the employed SMT solver, by default no timeout is used. The
  timeout is given in seconds.
* **SYMEX_TIMEBUDGET:** Time budget (in seconds) for path exploration,
  by default the software is explored until all branches have been
  negated.
* **SYMEX_TESTCASE:** This environment variable can point to a test case
  file for replaying inputs causing an error. This is most useful in
  conjunction with `--debug-mode`.

## How To Cite

The concepts behind SymEx-Trace-VP are further described in the following [publication](tbd):

## Acknowledgements
This work was supported in part by the German Federal Ministry of
Education and Research (BMBF) within the project Scale4Edge under contract
no. 16ME0127 and within the project VerSys under contract no. 01IW19001
and within the project ECXL.

The work on the original SymEx-VP was supported in part by the German Federal Ministry of
Education and Research (BMBF) within the project Scale4Edge under
contract no. 16ME0127 and within the project VerSys under contract
no. 01IW19001.

## License

The original riscv-vp code is licensed under MIT (see `LICENSE.MIT`).
All modifications made for the integration of symbolic execution 
with riscv-vp and the 
tracing extension 
are licensed under GPLv3+ (see `LICENSE.GPL`). Consult the
copyright headers of individual files for more information.

[riscv-vp github]: https://github.com/agra-uni-bremen/riscv-vp
[clover github]: https://github.com/agra-uni-bremen/clover
[wikipedia symex]: https://en.wikipedia.org/wiki/Symbolic_execution
[wikipedia ct]: https://en.wikipedia.org/wiki/Concolic_testing
[wikipedia mmio]: https://en.wikipedia.org/wiki/Memory-mapped_I/O
[systemc website]: https://systemc.org/
[gdb website]: https://www.gnu.org/software/gdb/
[docker website]: https://www.docker.io/
[make website]: https://www.gnu.org/software/make
[z3 repo]: https://github.com/Z3Prover/z3
[llvm website]: https://llvm.org/
[cmake website]: https://cmake.org/
[boost website]: https://www.boost.org/
[sifive hifive1]: https://www.sifive.com/boards/hifive1
[riot website]: https://riot-os.org/
[zephyr website]: https://zephyrproject.org/
[riscv-compliance github]: https://github.com/riscv/riscv-compliance/
[symex-vp paper]: https://doi.org/10.1016/j.sysarc.2022.102456
[symex-vp artifacts]: https://doi.org/10.24433/CO.7255660.v1
[systemc restart]: https://github.com/accellera-official/systemc/issues/8
[dac checkedc]: https://www.informatik.uni-bremen.de/agra/doc/konf/DAC-2021-CheckedC-Concolic-Testing.pdf
[fdl stack]: https://www.informatik.uni-bremen.de/agra/doc/konf/FDL21_VP_Stacksize.pdf

[SymEx-VP]:https://github.com/agra-uni-bremen/symex-vp
[SymEx-3D]:https://github.com/agra-uni-bremen/symex-3d
[3D SymEx Visualization]:tbd
[trace example]:https://github.com/agra-uni-bremen/symex-3d/blob/main/traces/v4/example_trace.rtrace
