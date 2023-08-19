#pragma once
#include <iostream>
#include <string.h>
#include <stdarg.h>

inline void ReportError(std::string ss)
{
	std::cout << " [Error] " << ss << std::endl;
}

inline void ReportWarning(std::string ss)
{
	std::cout << " [Warning] " << ss << std::endl;
}


inline std::string VFormat(const char* format, va_list ap)
{
	va_list zp;
	va_copy(zp, ap);
	char buffer[1024 * 10];
	vsnprintf(buffer, 1024 * 10, format, zp);
	va_end(zp);
	return buffer;
}

inline std::string Format(const char* format, ...)
{
	va_list va;
	va_start(va, format);
	std::string formatted = VFormat(format, va);
	va_end(va);
	return formatted;
}