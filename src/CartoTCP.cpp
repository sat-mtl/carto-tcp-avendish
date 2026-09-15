// CartoTCP is a header-only Avendish processor (CartoTCP.hpp + net/*.hpp): every
// binding compiles the header into its own translation unit. This TU exists only
// so the object library has a compiled member -- an otherwise source-less STATIC
// library is an empty archive, which GNU ar tolerates but macOS ar / MSVC lib
// reject ("no archive members specified"). It also gives a standalone check that
// the header compiles on its own.
#include "CartoTCP.hpp"
