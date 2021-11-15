include ./Make.defs

CFLAGS += -c -O3 -Wall -Wsequence-point -Wunused-function


BIN			= ../bin/ia_mpegh_dec_app_$(ARCH).out
LIBS		= ../lib/ia_mpegh_dec_app_$(ARCH).a

PATH_DEC      = ../../decoder
PATH_TEST      = ../../test
PATH_MP4_SRC   = ../../test/mp4/
PATH_MP4_INC   = ../../test/mp4/


SRCS+=$(PATH_TEST)/impeghd_error.c\
	  $(PATH_TEST)/impeghd_main.c\
	  $(PATH_MP4_SRC)/impeghd_mp4_file.c\
	  $(PATH_MP4_SRC)/impeghd_mp4_mae_parser.c\
	  $(PATH_MP4_SRC)/impeghd_mp4_file_wrapper.c\
	  $(PATH_MP4_SRC)/impeghd_mp4_init.c\
	  $(PATH_MP4_SRC)/impeghd_mp4_parser.c\
	  $(PATH_MP4_SRC)/impeghd_mp4_utils.c\
	  $(PATH_MP4_SRC)/impeghd_mp4_video.c
					
INCLUDES = 

ifneq ($(ARCH), armv7)					
INCLUDES += -I$(PATH_PLAT)
endif

INCLUDES += -I$(PATH_DEC) -I$(PATH_TEST) -I$(PATH_MP4_INC)

OBJS			= $(SRCS:.c=.o)
OBJS_LIBS		= $(SRCS_LIBS:.c=.o)

CFLAGS += $(INCLUDES)
all: $(BIN)

$(OBJS): %.o: %.c
	@echo CC `basename $<`
	@$(CC) $(CFLAGS) $*.c -o $*.o 
	
ifeq ($(ARCH), x86_64) 
$(BIN): $(OBJS) $(LIBS)
	@echo Building `basename $@`
	@$(CC) -o  $@ $^  -lm
	@rm -f $(OBJS)
else ifeq ($(ARCH), x86)	
$(BIN): $(OBJS) $(LIBS)
	@echo Building `basename $@`
	@$(CC) -m32 -o  $@ $^  -lm
	@rm -f $(OBJS)
else	
$(BIN): $(OBJS) $(LIBS)
	@echo Building `basename $@`
	@$(CC) -o  $@ $^ --static -lm	
	@rm -f $(OBJS)
endif	


clean: src_clean 		
	@rm -f $(BIN)
src_clean:
	@rm -f $(OBJS)

    
