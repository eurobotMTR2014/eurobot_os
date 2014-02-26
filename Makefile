#**********************************************************************************
#
#	Makefile - Projet Geppadi  - ECU
#
# Auteur G. Lejeune
#
#
#**********************************************************************************


#
# Project name
#
PROJ := eurobot

#
# Defines the part type that this project uses.
#
PART := LM3S2965

#
# OS name
#
OS	:= FreeRTOS

#
# Agent dir
#
AGENT := Agent

#
# Prefix of the compiling tools
#
PREFIX := arm-none-eabi

#
# Compiler
#
CC := $(PREFIX)-gcc

#
# Linker
#

LD = $(PREFIX)-ld

#
# Compiler options
#
CFLAGS =	-mthumb -mcpu=cortex-m3	-ffunction-sections	-fdata-sections -MD -std=c99 -Wall  -pedantic -DPART_${PART} -c -g -W -Wall


#	-nostdlib \
#					-L${LIBC}						\
#					-L${LIBGCC} 				\

OBJCOPY 		:= $(PREFIX)-objcopy

#
# Variables
#
SRCDIR 			:= src
BINDIR 			:= bin
DEPDIR 			:= dep
BACKUPDIR 	:= backups

OS_BINDIR		:= $(BINDIR)/$(OS)
OS_DIR			:= $(SRCDIR)/$(OS)

OS_INC			:= $(OS_DIR)
OS_INC			+= $(OS_DIR)/Include
OS_INC			+= $(OS_DIR)/ARM_CM3
OS_INC			+= $(SRCDIR)

AGENT_BINDIR 	:= $(BINDIR)/$(AGENT)
AGENT_DIR			:= $(SRCDIR)/$(AGENT)

AGENT_INC		:= $(OS_INC)
AGENT_INC		+= $(AGENT_DIR)
AGENT_INC 	+= $(SRCDIR)

SRC 				:= $(wildcard $(SRCDIR)/*.c)
OS_SRC 			:= $(wildcard $(OS_DIR)/*.c)
AGENT_SRC		:= $(wildcard $(AGENT_DIR)/*.c)

OBJ 				:= $(patsubst $(SRCDIR)/%.c,$(BINDIR)/%.o,$(SRC))
OBJ 				+= $(patsubst $(SRCDIR)/%.c,$(BINDIR)/%.o,$(OS_SRC))
OBJ 				+= $(patsubst $(SRCDIR)/%.c,$(BINDIR)/%.o,$(AGENT_SRC))

DATE 				:= $(shell date +%Y-%m-%d-%Hh%M)
PROJDIR 		:= $(shell basename $(shell pwd))

VERSION 		:= $(DATE)
DIST 				:= $(PROJDIR)/$(BACKUPDIR)/$(PROJ)-$(VERSION).tar.gz

MAKEFILE 		:= Makefile




#
# Get the location of libgcc.a from the GCC front-end.
#
LIBGCC=${shell ${CC} ${CFLAGS} -print-libgcc-file-name}

#
# Get the location of libc.a from the GCC front-end.
#
LIBC=${shell ${CC} ${CFLAGS} -print-file-name=libc.a}

LDRIVERS=$(SRCDIR)/driverlib/gcc/libdriver.a

#
# Linker options
#
LDFLAGS =	--gc-sections 			\
					--entry ResetISR \
					'$(LIBGCC)' \
			 		'$(LIBC)'\
					'$(LDRIVERS)'\


#
# Compiling all
#
all: $(PROJ).axf

#
# Linking
#
$(PROJ).axf: $(OBJ)
	@$(LD) -o $@ $^  $(LDFLAGS) -T src/lm3s2965.ld
	@echo $(LD) $@

	${OBJCOPY} -O binary ${@} ${@:.axf=.bin}




#
# Compiling
#
$(BINDIR)/%.o: $(SRCDIR)/%.c
		@mkdir -p $(BINDIR)
		@$(CC) $(CFLAGS) -o $@ -c $< $(foreach d, $(OS_INC), -I$d) $(foreach d, $(AGENT_INC), -I$d)
		@echo $(CC) $@


$(OS_BINDIR)%.o: $(OS_DIR)/%.c
		@mkdir -p $(OS_BINDIR)
		@$(CC) $(CFLAGS) -o $@ -c $< $(foreach d, $(OS_INC), -I$d)
		@echo $(CC) $@

$(AGENT_BINDIR)%.o: $(AGENT_DIR)/%.c
		@mkdir -p $(AGENT_BINDIR)
		@$(CC) $(CFLAGS) -o $@ -c $< $(foreach d, $(AGENT_INC), -I$d)
		@echo $(CC) $@

#
# File list for tarball and stats
#
FILES_LIST = $(wildcard doc $(MAKEFILE) $(SRCDIR)/*)


#
# Make an archive
#
dist: $(FILES_LIST)
	@mkdir -p $(BACKUPDIR)
	cd ..; tar -czhf $(DIST) $(addprefix $(PROJDIR)/,$^)



#
# Cleaning
#
clean:
		@rm -rf $(BINDIR)

mrproper: clean
		@rm  -f $(PROJ).axf $(PROJ).bin

.PHONY: clean mrproper
