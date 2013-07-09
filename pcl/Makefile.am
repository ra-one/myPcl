
INCLUDES = -I$(srcdir)/../include

lib_LTLIBRARIES = libpcl.la
libpcl_la_SOURCES = pcl.c pcl_version.c pcl_private.c \
	scc_config.c \
	memfun.c \
	RCCE_memcpy.c \
	scc.c \
	scc_comm_func.c \
	sccmalloc.c 
libpcl_la_LDFLAGS = -version-info 1:11:0 -lpthread

