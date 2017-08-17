/* RISC-V disassembler
   Copyright 2011-2015 Free Software Foundation, Inc.

   Contributed by Andrew Waterman (andrew@sifive.com).
   Based on MIPS target.

   This file is part of the GNU opcodes library.

   This library is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3, or (at your option)
   any later version.

   It is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
   or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
   License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; see the file COPYING3. If not,
   see <http://www.gnu.org/licenses/>.  */

#include "qemu/osdep.h"
#include "disas/bfd.h"

#define MATCH_FIELD
#include "disas/zpu-opc.h"
#undef MATCH_FIELD

/*
 * definition from include/opcode/zpu.h
 */
typedef uint64_t insn_t;

static inline unsigned int zpu_insn_length (insn_t insn)
{
    return 4;
}

static const char * const zpu_rm[8] = {
};
static const char * const zpu_pred_succ[16] = {
};

#define OP_MASK_OP		0x7f
#define OP_MASK_RD		0x1f


#define NGPR 32
#define NFPR 32

/* Replace bits MASK << SHIFT of STRUCT with the equivalent bits in
   VALUE << SHIFT.  VALUE is evaluated exactly once.  */
#define INSERT_BITS(STRUCT, VALUE, MASK, SHIFT) \
  (STRUCT) = (((STRUCT) & ~((insn_t)(MASK) << (SHIFT))) \
	      | ((insn_t)((VALUE) & (MASK)) << (SHIFT)))

/* Extract bits MASK << SHIFT from STRUCT and shift them right
   SHIFT places.  */
#define EXTRACT_BITS(STRUCT, MASK, SHIFT) \
  (((STRUCT) >> (SHIFT)) & (MASK))

/* Extract the operand given by FIELD from integer INSN.  */
#define EXTRACT_OPERAND(FIELD, INSN) \
  EXTRACT_BITS ((INSN), OP_MASK_##FIELD, OP_SH_##FIELD)

/* This structure holds information for a particular instruction.  */

struct zpu_opcode
{
  /* The name of the instruction.  */
  const char *name;
  /* The ISA subset name (I, M, A, F, D, Xextension).  */
  const char *subset;
  /* A string describing the arguments for this instruction.  */
  const char *args;
  /* The basic opcode for the instruction.  When assembling, this
     opcode is modified by the arguments to produce the actual opcode
     that is used.  If pinfo is INSN_MACRO, then this is 0.  */
  insn_t match;
  /* If pinfo is not INSN_MACRO, then this is a bit mask for the
     relevant portions of the opcode when disassembling.  If the
     actual opcode anded with the match field equals the opcode field,
     then we have found the correct instruction.  If pinfo is
     INSN_MACRO, then this field is the macro identifier.  */
  insn_t mask;
  /* A function to determine if a word corresponds to this instruction.
     Usually, this computes ((word & mask) == match).  */
  int (*match_func) (const struct zpu_opcode *op, insn_t word);
  /* For a macro, this is INSN_MACRO.  Otherwise, it is a collection
     of bits describing the instruction, notably any relevant hazard
     information.  */
  unsigned long pinfo;
};

/* Instruction is a simple alias (e.g. "mv" for "addi").  */
#define	INSN_ALIAS		0x00000001
/* Instruction is actually a macro.  It should be ignored by the
   disassembler, and requires special treatment by the assembler.  */
#define INSN_MACRO		0xffffffff

/* This is a list of macro expanded instructions.

   _I appended means immediate
   _A appended means address
   _AB appended means address with base register
   _D appended means 64 bit floating point constant
   _S appended means 32 bit floating point constant.  */

enum
{
  M_LA,
};


/*
 * definition from opcode/zpu-opc.c
 */

/* Register names used by gas and objdump.  */
const char * const zpu_gpr_names_numeric[NGPR] =
{
};

const char * const zpu_gpr_names_abi[NGPR] = {
};


const struct zpu_opcode zpu_opcodes[] =
{
/* Terminate the list.  */
{0, 0, 0, 0, 0, 0, 0}
};

/*
 * definiton from opcode/zpu-dis.c
 */

struct zpu_private_data
{
  bfd_vma gp;
  bfd_vma print_addr;
  bfd_vma hi_addr[OP_MASK_RD + 1];
};

static const char * const *zpu_gpr_names;

/* Other options */
static int no_aliases;	/* If set disassemble as most general inst.  */

static void
set_default_zpu_dis_options (void)
{
  zpu_gpr_names = zpu_gpr_names_abi;
  no_aliases = 0;
}

#if 0
static void
parse_zpu_dis_option (const char *option)
{
  if (strcmp (option, "no-aliases") == 0)
    no_aliases = 1;
  else if (strcmp (option, "numeric") == 0)
    {
      zpu_gpr_names = zpu_gpr_names_numeric;
      zpu_fpr_names = zpu_fpr_names_numeric;
    }
  else
    {
      /* Invalid option.  */
      fprintf (stderr, _("Unrecognized disassembler option: %s\n"), option);
    }
}

static void
parse_zpu_dis_options (const char *opts_in)
{
  char *opts = xstrdup (opts_in), *opt = opts, *opt_end = opts;

  set_default_zpu_dis_options ();

  for ( ; opt_end != NULL; opt = opt_end + 1)
    {
      if ((opt_end = strchr (opt, ',')) != NULL)
	*opt_end = 0;
      parse_zpu_dis_option (opt);
    }

  free (opts);
}
#endif


/* Print insn arguments for 32/64-bit code.  */

static void
print_insn_args (const char *d, insn_t l, bfd_vma pc, disassemble_info *info)
{
}

/* Print the RISC-V instruction at address MEMADDR in debugged memory,
   on using INFO.  Returns length of the instruction, in bytes.
   BIGENDIAN must be 1 if this is big-endian code, 0 if
   this is little-endian code.  */

static int
zpu_disassemble_insn (bfd_vma memaddr, insn_t word, disassemble_info *info)
{
  const struct zpu_opcode *op;
  static bfd_boolean init = 0;
  static const struct zpu_opcode *zpu_hash[OP_MASK_OP + 1];
  struct zpu_private_data *pd;
  int insnlen;

#define OP_HASH_IDX(i) ((i) & (zpu_insn_length (i) == 2 ? 0x3 : OP_MASK_OP))

  /* Build a hash table to shorten the search time.  */
  if (! init)
    {
      for (op = zpu_opcodes; op->name; op++)
	if (!zpu_hash[OP_HASH_IDX (op->match)])
	  zpu_hash[OP_HASH_IDX (op->match)] = op;

      init = 1;
    }

  if (info->private_data == NULL)
    {
      int i;

      pd = info->private_data = g_malloc0(sizeof (struct zpu_private_data));
      pd->gp = -1;
      pd->print_addr = -1;
      for (i = 0; i < (int)ARRAY_SIZE (pd->hi_addr); i++)
	pd->hi_addr[i] = -1;

#if 0
      for (i = 0; i < info->symtab_size; i++)
	if (strcmp (bfd_asymbol_name (info->symtab[i]), "_gp") == 0)
	  pd->gp = bfd_asymbol_value (info->symtab[i]);
#endif
    }
  else
    pd = info->private_data;

  insnlen = zpu_insn_length (word);

  info->bytes_per_chunk = insnlen % 4 == 0 ? 4 : 2;
  info->bytes_per_line = 8;
  info->display_endian = info->endian;
  info->insn_info_valid = 1;
  info->branch_delay_insns = 0;
  info->data_size = 0;
  info->insn_type = dis_nonbranch;
  info->target = 0;
  info->target2 = 0;

  op = zpu_hash[OP_HASH_IDX (word)];
  if (op != NULL)
    {
      int xlen = 0;

#if 0
      /* The incoming section might not always be complete.  */
      if (info->section != NULL)
	{
	  Elf_Internal_Ehdr *ehdr = elf_elfheader (info->section->owner);
	  xlen = ehdr->e_ident[EI_CLASS] == ELFCLASS64 ? 64 : 32;
	}
#endif

      for (; op->name; op++)
	{
	  /* Does the opcode match?  */
	  if (! (op->match_func) (op, word))
	    continue;
	  /* Is this a pseudo-instruction and may we print it as such?  */
	  if (no_aliases && (op->pinfo & INSN_ALIAS))
	    continue;
	  /* Is this instruction restricted to a certain value of XLEN?  */
	  if (isdigit (op->subset[0]) && atoi (op->subset) != xlen)
	    continue;

	  /* It's a match.  */
	  (*info->fprintf_func) (info->stream, "%s", op->name);
	  print_insn_args (op->args, word, memaddr, info);

#if 0
	  /* Try to disassemble multi-instruction addressing sequences.  */
	  if (pd->print_addr != (bfd_vma)-1)
	    {
	      info->target = pd->print_addr;
	      (*info->fprintf_func) (info->stream, " # ");
	      (*info->print_address_func) (info->target, info);
	      pd->print_addr = -1;
	    }
#endif

	  return insnlen;
	}
    }

  /* We did not find a match, so just print the instruction bits.  */
  info->insn_type = dis_noninsn;
  (*info->fprintf_func) (info->stream, "0x%llx", (unsigned long long)word);
  return insnlen;
}

int
print_insn_zpu (bfd_vma memaddr, struct disassemble_info *info)
{
  bfd_byte packet[2];
  insn_t insn = 0;
  bfd_vma n;
  int status;

#if 0
  if (info->disassembler_options != NULL)
    {
      parse_zpu_dis_options (info->disassembler_options);
      /* Avoid repeatedly parsing the options.  */
      info->disassembler_options = NULL;
    }
  else if (zpu_gpr_names == NULL)
#endif
    set_default_zpu_dis_options ();

  /* Instructions are a sequence of 2-byte packets in little-endian order.  */
  for (n = 0; n < sizeof (insn) && n < zpu_insn_length (insn); n += 2)
    {
      status = (*info->read_memory_func) (memaddr + n, packet, 2, info);
      if (status != 0)
	{
	  /* Don't fail just because we fell off the end.  */
	  if (n > 0)
	    break;
	  (*info->memory_error_func) (status, memaddr, info);
	  return status;
	}

      insn |= ((insn_t) bfd_getl16 (packet)) << (8 * n);
    }

  return zpu_disassemble_insn (memaddr, insn, info);
}

#if 0
void
print_zpu_disassembler_options (FILE *stream)
{
  fprintf (stream, _("\n\
The following RISC-V-specific disassembler options are supported for use\n\
with the -M switch (multiple options should be separated by commas):\n"));

  fprintf (stream, _("\n\
  numeric       Print numeric reigster names, rather than ABI names.\n"));

  fprintf (stream, _("\n\
  no-aliases    Disassemble only into canonical instructions, rather\n\
                than into pseudoinstructions.\n"));

  fprintf (stream, _("\n"));
}
#endif
