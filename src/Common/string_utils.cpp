#include "string_utils.h"


#include <memory>
#include <stdarg.h>

#include <string>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <assert.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>
using namespace boost::uuids;
using namespace std;


#if defined(WIN32)||defined(WINCE)||defined(WIN64)
#include <objbase.h>
#endif

namespace insight{
std::string GetUUID()
{
#if defined(WIN32)||defined(WINCE)||defined(WIN64)
#define GUID_LEN 64
	std::string strUUID;
     char buffer[GUID_LEN] = { 0 };
     GUID guid;

     if ( CoCreateGuid(&guid) )
     {
      fprintf(stderr, "create guid error\n");
      return "";
     }
     _snprintf(buffer, sizeof(buffer),
      "%08X%04X%04x%02X%02X%02X%02X%02X%02X%02X%02X",
      guid.Data1, guid.Data2, guid.Data3,
      guid.Data4[0], guid.Data4[1], guid.Data4[2],
      guid.Data4[3], guid.Data4[4], guid.Data4[5],
      guid.Data4[6], guid.Data4[7]);
     strUUID = buffer;
	 return strUUID;
#else
	random_generator rgen;//���������
	uuid u = rgen();
	string str = boost::lexical_cast<string>(u);  //uuidת�����ַ���
	return str;
#endif

}

void StringAppendV(std::string* dst, const char* format, va_list ap)
{
	// First try with a small fixed size buffer.
	static const int kFixedBufferSize = 1024;
	char fixed_buffer[kFixedBufferSize];

	// It is possible for methods that use a va_list to invalidate
	// the data in it upon use.  The fix is to make a copy
	// of the structure before using it and use that copy instead.
	va_list backup_ap;
	va_copy(backup_ap, ap);
	int result = vsnprintf(fixed_buffer, kFixedBufferSize, format, backup_ap);
	va_end(backup_ap);

	if (result < kFixedBufferSize) {
		if (result >= 0) {
			// Normal case - everything fits.
			dst->append(fixed_buffer, result);
			return;
		}

#ifdef _MSC_VER
		// Error or MSVC running out of space.  MSVC 8.0 and higher
		// can be asked about space needed with the special idiom below:
		va_copy(backup_ap, ap);
		result = vsnprintf(nullptr, 0, format, backup_ap);
		va_end(backup_ap);
#endif

		if (result < 0) {
			// Just an error.
			return;
		}
	}

	// Increase the buffer size to the size requested by vsnprintf,
	// plus one for the closing \0.
	const int variable_buffer_size = result + 1;
	std::unique_ptr<char> variable_buffer(new char[variable_buffer_size]);

	// Restore the va_list before we use it again.
	va_copy(backup_ap, ap);
	result =
		vsnprintf(variable_buffer.get(), variable_buffer_size, format, backup_ap);
	va_end(backup_ap);

	if (result >= 0 && result < variable_buffer_size) {
		dst->append(variable_buffer.get(), result);
	}
}

std::string StringPrintf(const char* format, ...)
{
	va_list ap;
	va_start(ap, format);
	std::string result;
	StringAppendV(&result, format, ap);
	va_end(ap);
	return result;
}

}//name space insight
