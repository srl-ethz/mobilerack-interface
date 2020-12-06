# [{fmt} library](https://github.com/fmtlib/fmt)
allows easier formatting of strings, e.g.
```c++
std::string s = fmt::format("The answer is {}.", 42);
// s == "The answer is 42."
```

`std::format` is natively supported in C++20, but in the meantime use this library