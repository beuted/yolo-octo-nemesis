#ifndef __DOSUNIX_BIND_H__
#define __DOSUNIX_BIND_H__


#ifdef WIN32
	#define strtok_r_dosunix(str, delim, saveptr) strtok_s(str, delim, saveptr)
#else
	#define strtok_r_dosunix(str, delim, saveptr) strtok_r(str, delim, saveptr)
#endif


#ifdef WIN32
	#define fopen_r_dosunix(stream, path, mode) fopen_r(stream, path, mode)
#else
	#define fopen_r_dosunix(stream, path, mode) freopen(path, mode, *stream)
#endif


#ifdef WIN32
	#define sscanf_dosunix(str, format, ...) sscanf_s(str, format, __VA_ARGS__)
#else
	#define sscanf_dosunix(str, format, ...) sscanf(str, format, __VA_ARGS__)
#endif


#ifdef WIN32
	#define sprintf_s_dosunix(buffer, format, ...) sprintf_s(buffer, format, __VA_ARGS__)
#else
	#define sprintf_s_dosunix(buffer, format, ...) sprintf( buffer, format, __VA_ARGS__)
#endif


#endif /* end of include guard: __DOSUNIX_BIND_H__ */
