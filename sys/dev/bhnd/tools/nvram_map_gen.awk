#!/usr/bin/awk -f

#-
# Copyright (c) 2015-2016 Landon Fuller <landon@landonf.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer,
#    without modification.
# 2. Redistributions in binary form must reproduce at minimum a disclaimer
#    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
#    redistribution must be conditioned upon including a substantially
#    similar Disclaimer requirement for further binary redistribution.
#
# NO WARRANTY
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
# AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
# THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
# OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
# IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGES.
# 
# $FreeBSD$

BEGIN {
	RS="\n"

	depth = 0
	symbols[depth,"_file"] = FILENAME
	num_output_vars = 0
	OUTPUT_FILE = null

	# Seed rand()
	srand()

	# Output type
	OUT_T = null
	OUT_T_HEADER = "HEADER"
	OUT_T_DATA = "DATA"

	# Enable debug output
	DEBUG = 0

	# Maximum revision
	REV_MAX = 255

	# Parse arguments
	if (ARGC < 2)
		usage()

	for (i = 1; i < ARGC; i++) {
		if (ARGV[i] == "--debug") {
			DEBUG = 1
		} else if (ARGV[i] == "-d" && OUT_T == null) {
			OUT_T = OUT_T_DATA
		} else if (ARGV[i] == "-h" && OUT_T == null) {
			OUT_T = OUT_T_HEADER
		} else if (ARGV[i] == "-o") {
			i++
			if (i >= ARGC)
				usage()

			OUTPUT_FILE = ARGV[i]
		} else if (ARGV[i] == "--") {
			i++
			break
		} else if (ARGV[i] !~ /^-/) {
			FILENAME = ARGV[i]
		} else {
			print "unknown option " ARGV[i]
			usage()
		}
	}

	ARGC=2

	if (OUT_T == null) {
		print("error: one of -d or -h required")
		usage()
	}

	if (FILENAME == null) {
		print("error: no input file specified")
		usage()
	}

	if (OUTPUT_FILE == "-") {
		OUTPUT_FILE = "/dev/stdout"
	} else if (OUTPUT_FILE == null) {
		_bi = split(FILENAME, _paths, "/")
		OUTPUT_FILE = _paths[_bi]

		if (OUTPUT_FILE !~ /^bhnd_/)
			OUTPUT_FILE = "bhnd_" OUTPUT_FILE

		if (OUT_T == OUT_T_HEADER)
			OUTPUT_FILE = OUTPUT_FILE ".h" 
		else
			OUTPUT_FILE = OUTPUT_FILE "_data.h"
	}

	# Format Constants
	FMT["hex"]	= "BHND_NVRAM_SFMT_HEX"
	FMT["decimal"]	= "BHND_NVRAM_SFMT_DEC"
	FMT["ccode"]	= "BHND_NVRAM_SFMT_CCODE"
	FMT["macaddr"]	= "BHND_NVRAM_SFMT_MACADDR"
	FMT["led_dc"]	= "BHND_NVRAM_SFMT_LEDDC"

	# Data Type Constants
	DTYPE["u8"]	= "BHND_NVRAM_TYPE_UINT8"
	DTYPE["u16"]	= "BHND_NVRAM_TYPE_UINT16"
	DTYPE["u32"]	= "BHND_NVRAM_TYPE_UINT32"
	DTYPE["i8"]	= "BHND_NVRAM_TYPE_INT8"
	DTYPE["i16"]	= "BHND_NVRAM_TYPE_INT16"
	DTYPE["i32"]	= "BHND_NVRAM_TYPE_INT32"
	DTYPE["char"]	= "BHND_NVRAM_TYPE_CHAR"

	# Default masking for standard types
	TMASK["u8"]	= "0x000000FF"
	TMASK["u16"]	= "0x0000FFFF"
	TMASK["u32"]	= "0xFFFFFFFF"
	TMASK["i8"]	= TMASK["u8"]
	TMASK["i16"]	= TMASK["u16"]
	TMASK["i32"]	= TMASK["u32"]
	TMASK["char"]	= TMASK["u8"]

	# Byte sizes for standard types
	TSIZE["u8"]	= "1"
	TSIZE["u16"]	= "2"
	TSIZE["u32"]	= "4"
	TSIZE["i8"]	= TSIZE["u8"]
	TSIZE["i16"]	= TSIZE["u8"]
	TSIZE["i32"]	= TSIZE["u8"]
	TSIZE["char"]	= "1"

	# Common Regexs
	INT_REGEX	= "^(0|[1-9][0-9]*),?$"
	HEX_REGEX	= "^0x[A-Fa-f0-9]+,?$"

	ARRAY_REGEX	= "\\[(0|[1-9][0-9]*)\\]"
	TYPES_REGEX	= "^(((u|i)(8|16|32))|char)("ARRAY_REGEX")?,?$"

	IDENT_REGEX	= "^[A-Za-z_][A-Za-z0-9_]*,?$"
	SROM_OFF_REGEX	= "("TYPES_REGEX"|"HEX_REGEX")"

	# Parser states types
	ST_STRUCT_BLOCK	= "struct"	# struct block
	ST_VAR_BLOCK	= "var"		# variable block
	ST_SROM_DEFN	= "srom"	# srom offset defn
	ST_NONE		= "NONE"	# default state

	# Property types
	PROP_T_SFMT	= "sfmt"
	PROP_T_ALL1	= "all1"

	# Internal variables used for parser state
	# tracking
	STATE_TYPE	= "_state_type"
	STATE_IDENT	= "_state_block_name"
	STATE_OBJ	= "_state_obj"
	STATE_LINENO	= "_state_first_line"
	STATE_ISBLOCK	= "_state_is_block"

	# Common array keys
	DEF_LINE	= "def_line"
	NUM_REVS	= "num_revs"
	REV		= "rev"

	# Revision array keys
	REV_START	= "rev_start"
	REV_END		= "rev_end"
	REV_DESC	= "rev_decl"
	REV_NUM_OFFS	= "num_offs"

	# Offset array keys
	OFF 		= "off"
	OFF_NUM_SEGS	= "off_num_segs"
	OFF_SEG		= "off_seg"

	# Segment array keys
	SEG_ADDR	= "seg_addr"
	SEG_COUNT	= "seg_count"
	SEG_TYPE	= "seg_type"
	SEG_MASK	= "seg_mask"
	SEG_SHIFT	= "seg_shift"

	# Variable array keys
	VAR_NAME	= "v_name"
	VAR_TYPE	= "v_type"
	VAR_BASE_TYPE	= "v_base_type"
	VAR_FMT		= "v_fmt"
	VAR_STRUCT	= "v_parent_struct"
	VAR_PRIVATE	= "v_private"
	VAR_ARRAY	= "v_array"
	VAR_IGNALL1	= "v_ignall1"

	# Property array keys
	PROP_ID		= "p_id"
	PROP_NAME	= "p_name"

	# Object array keys
	OBJ_IS_CLS	= "o_is_cls"
	OBJ_SUPER	= "o_super"
	OBJ_PROP	= "o_prop"

	# Class array keys
	CLS_NAME	= "cls_name"
	CLS_PROP	= "cls_prop"

	# List class definition
	List = class_new("List")
		class_add_prop(List, _head, "head")
		class_add_prop(List, _tail, "tail")

	ListNode = class_new("ListNode")
		class_add_prop(ListNode, _v, "v")
		class_add_prop(ListNode, _next, "next")
		class_add_prop(ListNode, _prev, "prev")

	# ParseState definition
	ParseState = class_new("ParseState")
		class_add_prop(ParseState, _ctx, "ctx")
		class_add_prop(ParseState, _is_block, "is_block")
		class_add_prop(ParseState, _line, "line")

	# VarFlag class definition
	VarFlag = class_new("VarFlag")
		class_add_prop(VarFlag, _name, "name")
		class_add_prop(VarFlag, _const, "const")

	# Type class definitions
	Type = class_new("Type")
		class_add_prop(Type, _name, "name")
		class_add_prop(Type, _width, "width")
		class_add_prop(Type, _const, "const")
		class_add_prop(Type, _default_fmt, "default_fmt")
		class_add_prop(Type, _mask, "mask")

	ArrayType = class_new("ArrayType", AbstractType)
		class_add_prop(ArrayType, _type, "type")
		class_add_prop(ArrayType, _count, "count")

	# SFmt class definition
	SFmt = class_new("SFmt")
		class_add_prop(SFmt, _name, "name")
		class_add_prop(SFmt, _const, "const")

	# AST class definitions
	AST = class_new("AST")
		class_add_prop(AST, _line, "line")

	SymbolContext = class_new("SymbolContext", AST)
		class_add_prop(SymbolContext, _vars, "vars")

	# Root symbol context
	RootContext = class_new("RootContext", SymbolContext)
		class_add_prop(RootContext, _structs, "structs")

	RevSet = class_new("RevSet", AST)
		class_add_prop(RevSet, _start, "start")
		class_add_prop(RevSet, _end, "end")

	Offset = class_new("Offset", AST)
		class_add_prop(Offset, _segments, "segments")

	SROM = class_new("SROM", AST)
		class_add_prop(RevSet, _revisions, "revisions")
		class_add_prop(RevSet, _offsets, "offsets")

	Segment = class_new("Segment", AST)
		class_add_prop(Segment, _addr, "addr")
		class_add_prop(Segment, _type, "type")
		class_add_prop(Segment, _mask, "mask")
		class_add_prop(Segment, _shift, "shift")

	Var = class_new("Var", AST)
		class_add_prop(Var, _name, "name")
		class_add_prop(Var, _type, "type")
		class_add_prop(Var, _fmt, "fmt")
		class_add_prop(Var, _flags, "flags")
		class_add_prop(Var, _offsets, "offsets")

	Struct = class_new("Struct", SymbolContext)
		class_add_prop(Struct, _name, "name")
		class_add_prop(Struct, _offsets, "offsets")

	# Type constants
	UInt8	= type_new("u8", 1, "BHND_NVRAM_TYPE_UINT8", SFmtHex,
	    "0x000000FF")
	UInt16	= type_new("u16", 2, "BHND_NVRAM_TYPE_UINT16", SFmtHex,
	    "0x0000FFFF")
	UInt32	= type_new("u32", 4, "BHND_NVRAM_TYPE_UINT32", SFmtHex,
	    "0xFFFFFFFF")
	Int8	= type_new("i8", 1, "BHND_NVRAM_TYPE_INT8", SFmtDec,
	    "0x000000FF")
	Int16	= type_new("i16", 2, "BHND_NVRAM_TYPE_INT16", SFmtDec,
	    "0x0000FFFF")
	Int32	= type_new("i32", 4, "BHND_NVRAM_TYPE_INT32", SFmtDec,
	    "0xFFFFFFFF")
	Char	= type_new("char", 1, "BHND_NVRAM_TYPE_CHAR", SFmtHex,
	    "0x000000FF")

	BaseTypes = list_new()
		lappend(BaseTypes, UInt8)
		lappend(BaseTypes, UInt16)
		lappend(BaseTypes, UInt32)
		lappend(BaseTypes, Int8)
		lappend(BaseTypes, Int16)
		lappend(BaseTypes, Int32)
		lappend(BaseTypes, Char)

	# Variable flag constants
	VarPrivate	= var_flag_new("private", "BHND_NVRAM_VF_MFGINT")
	IgnAll1		= var_flag_new("ignall1", "BHND_NVRAM_VF_IGNALL1")

	# String format constants
	SFmtHex		= sfmt_new("hex", "BHND_NVRAM_SFMT_HEX")
	SFmtDec 	= sfmt_new("decimal", "BHND_NVRAM_SFMT_DEC")
	SFmtCCODE	= sfmt_new("ccode", "BHND_NVRAM_SFMT_CCODE")
	SFmtMAC		= sfmt_new("macaddr", "BHND_NVRAM_SFMT_MACADDR")
	SFmtLEDDC	= sfmt_new("led_dc", "BHND_NVRAM_SFMT_LEDDC")

	StringFormats = list_new()
		lappend(StringFormats, SFmtHex)
		lappend(StringFormats, SFmtDec)
		lappend(StringFormats, SFmtCCODE)
		lappend(StringFormats, SFmtMAC)
		lappend(StringFormats, SFmtLEDDC)

	# Create the parse state stack
	_g_parse_stack = list_new()

	# Push the root parse state
	_g_root_ctx = obj_new(RootContext)
	set(_g_root_ctx, _vars, list_new())
	set(_g_root_ctx, _structs, list_new())

	parser_state_push(_g_root_ctx, 0)
}

# Create a class instance with the given name
function class_new (name, superclass, _class)
{
	if (_class != null)
		errorx("class_get() must be called with one or two arguments")

	# Look for an existing class instance
	if (name in _g_class_names)
		errorx("redefining class: " name)

	# Create and register the class object
	_class = obj_new(superclass)
	_g_class_names[name] = _class
	_g_obj[_class,OBJ_IS_CLS] = 1
	_g_obj[_class,CLS_NAME] = name

	return (_class)
}

# Return the class instance with the given name
function class_get (name)
{
	if (name in _g_class_names)
		return (_g_class_names[name])

	errorx("no such class " name)
}

# Return the name of cls
function class_get_name (cls)
{
	if (cls == null) {
		warnx("class_get_name() called with null class")
		return "<null>"
	}

	if (!obj_is_class(cls))
		errorx(cls " is not a class object")

	return (_g_obj[cls,CLS_NAME])
}

# Return true if the given property prop is defined on class
function class_has_property (class, prop, _super, _prop_id)
{
	if (_super != null || _prop_id != null)
		errorx("class_has_property() must be called with two arguments")

	if (class == null)
		return (false)

	if (!obj_is_class(class))
		errorx(class " is not a class object")

	if (!(PROP_ID in prop))
		return (false)

	for (_super = class; _super != null; _super = obj_get_class(_super))
	{
		if ((_super,CLS_PROP,prop[PROP_ID]) in _g_obj)
			return (1)
	}

	return (0)
}

# Define a `prop` on `class` with the given `name` string
function class_add_prop (class, prop, name, _prop_id)
{
	if (_prop_id != null)
		errorx("class_add_prop() must be called with three arguments")

	# Check for duplicate property definition
	if (class_has_property(class, prop))
		errorx("property " prop[PROP_NAME] " already defined on " \
		    class_get_name(class))

	# Init property IDs
	if (_g_prop_ids == null)
		_g_prop_ids = 1

	# Get (or create) new property entry
	if (name in _g_prop_names) {
		_prop_id = _g_prop_names[name]
	} else {
		_prop_id = _g_prop_ids++
		_g_prop_names[name] = _prop_id
		_g_props[_prop_id] = name

		prop[PROP_NAME]	= name
		prop[PROP_ID]	= _prop_id
	}

	# Add to class definition
	_g_obj[class,CLS_PROP,prop[PROP_ID]] = name
	return (name)
}

# Create a new instance of the given class
function obj_new (class, _obj)
{
	if (_obj != null)
		errorx("obj_new() must be called with one argument")

	if (_g_obj_ids == null)
		_g_obj_ids = 1

	# Assign ID and set superclass
	_obj = _g_obj_ids++
	_g_obj[_obj,OBJ_SUPER] = class

	return (_obj)
}

# Destroy all metadata associated with the given object
function obj_delete (obj, _key)
{
	if (obj_is_class(obj))
		errorx("cannot delete class objects")

	for (_key in _g_obj) {
                if (obj != substr(_key, 0, index(_key, SUBSEP) - 1))
                        continue
                delete _g_obj[_key]
	}
}

# Return true if obj is a class object
function obj_is_class (obj)
{
	return (_g_obj[obj,OBJ_IS_CLS] == 1)
}

# Return the class of obj, if any.
function obj_get_class (obj)
{
	if (obj == null)
		errorx("obj_get_class() on null object")
	return (_g_obj[obj,OBJ_SUPER])
}

# Return true if obj is an instance of the given class
function obj_is_instanceof (obj, class, _super)
{
	if (_super != null)
		errorx("obj_is_instanceof() must be called with two arguments")

	if (!obj_is_class(class))
		errorx(class " is not a class object")

	if (obj == null) {
		errorx("obj_is_instanceof() called with null obj (class " \
		    class_get_name(class) ")")
	}

	for (_super = obj_get_class(obj); _super != null;
	     _super = obj_get_class(_super))
	{
		if (_super == class)
			return (1)
	}

	return (0)
}

# Assert that obj is an instance of the given class
function obj_assert_class (obj, class)
{
	if (!obj_is_instanceof(obj, class)) {
		print "OBJ: " obj " class '" obj_get_class(obj) "'"
		errorx(class_get_name(obj_get_class(obj)) "<" obj "> is not " \
		    "an instance of " class_get_name(class))
	}
}

# Set a property on obj
function set(obj, prop, value, _class)
{
	if (obj == null)
		errorx("setting property '"prop[PROP_NAME]"' on null object")

	_class = obj_get_class(obj)
	if (_class == null)
		errorx(obj " has no superclass")
	
	if (!class_has_property(_class, prop)) {
		warn("requested undefined property on " class_get_name(_class))
		errorx("undefined property '" prop "'")
	}

	_g_obj[obj,OBJ_PROP,prop[PROP_ID]] = value
}

# Get a property defined on obj
function get(obj, prop, _class)
{
	if (obj == null)
		errorx("requested property '"prop[PROP_NAME]"' on null object")

	_class = obj_get_class(obj)
	if (_class == null)
		errorx(obj " has no superclass")

	if (!class_has_property(_class, prop)) {
		warn("requested undefined property on " class_get_name(_class))
		errorx("undefined property '" prop "'")
	}

	return (_g_obj[obj,OBJ_PROP,prop[PROP_ID]])
}

# Create a new Type instance
function type_new (name, width, constant, fmt, mask, _obj)
{
	_obj = obj_new(Type)
	set(_obj, _name, name)
	set(_obj, _width, width)
	set(_obj, _const, constant)
	set(_obj, _default_fmt, fmt)
	set(_obj, _mask, mask)

	return (_obj)
}

# Return the default format for a given type / array type
function type_get_default_format (type)
{
	if (obj_is_instanceof(type, ArrayType))
		return (type_get_default_format(get(type, _type)))

	obj_assert_class(type, Type)
	return (get(type, _default_fmt))
}

# Return a string representation of the given type
function type_desc (type)
{
	if (obj_is_instanceof(type, ArrayType))
		return (type_desc(get(type, _type)) "[" get(type, _count) "]")

	obj_assert_class(type, Type)
	return get(type, _name)
}

# Create a new ArrayType instance
function array_type_new (type, count, _obj)
{
	_obj = obj_new(ArrayType)
	set(_obj, _type, type)
	set(_obj, _count, count)

	return (_obj)
}

# Create a new SFmt instance
function sfmt_new (name, constant, _obj)
{
	_obj = obj_new(SFmt)
	set(_obj, _name, name)
	set(_obj, _const, constant)

	return (_obj)
}

# Create an empty list
function list_new ()
{
	return (obj_new(List))
}

# Return true if the list is empty
function list_empty (list)
{
	return (lhead(list) == null)
}

function list_dump (list, _node)
{
	obj_assert_class(list, List)
	print "List<" list ">"
	print "head: " get(list, _head)
	print "tail: " get(list, _tail)

	for (_node = lhead(list); _node != null; _node = lnext(_node)) {
		obj_assert_class(_node, ListNode)
		print "node: " _node
		print "\tprev: " get(_node, _prev)
		print "\tnext: " get(_node, _next)
		print "\tvalue: " get(_node, _v)
	}
}

# Append value to list
function lappend (list, value, _node, _cur)
{
	obj_assert_class(list, List)

	# Create the new node
	_node = obj_new(ListNode)
	set(_node, _v, value)

	# Insert in list
	_cur = get(list, _tail)

	if (_cur == null) {
		# Insert at head of list
		set(list, _head, _node)
		set(list, _tail, _node)		
	} else {
		set(_node, _prev, _cur)
		set(_cur, _next, _node)

		set(list, _tail, _node)
	}
}

# Return the first node in list, or null
function lhead (list)
{
	obj_assert_class(list, List)
	return (get(list, _head))
}

# Return the last node in list, or null
function ltail (list)
{
	obj_assert_class(list, List)
	return (get(list, _tail))
}

# Return the next element in the list
function lnext (lnode)
{
	obj_assert_class(lnode, ListNode)
	return (get(lnode, _next))
}

# Remove a list node from its enclosing list
function ldelete (list, lnode, _node_prev, _node_next)
{
	obj_assert_class(list, List)
	obj_assert_class(lnode, ListNode)

	_node_prev = get(lnode, _prev)
	_node_next = get(lnode, _next)

	if (_node_prev != null)
		set(_node_prev, _next, _node_next)

	if (_node_next != null)
		set(_node_next, _prev, _node_prev)

	if (get(list, _head) == lnode)
		set(list, _head, _node_next)

	if (get(list, _tail) == lnode)
		set(list, _tail, _node_prev)

	obj_delete(lnode)
}

# Return the value associated with the given list node
function lvalue (lnode)
{
	obj_assert_class(lnode, ListNode)
	return (get(lnode, _v))
}

# Create a new VarFlag instance
function var_flag_new (name, constant, _obj)
{
	_obj = obj_new(VarFlag)
	set(_obj, _name, name)
	set(_obj, _const, constant)

	return (_obj)
}

# Create a new Var instance
function var_new (name, type, fmt, flags, line, _obj)
{
	_obj = obj_new(Var)
	set(_obj, _name, name)
	set(_obj, _type, type)
	set(_obj, _fmt, fmt)
	set(_obj, _flags, flags)
	set(_obj, _offsets, list_new())
	set(_obj, _line, line)

	return (_obj)
}

# return the flag definition for variable `v`
function gen_var_flags (v)
{
	_num_flags = 0;
	if (vars[v,VAR_ARRAY])
		_flags[_num_flags++] = "BHND_NVRAM_VF_ARRAY"

	if (vars[v,VAR_PRIVATE])
		_flags[_num_flags++] = "BHND_NVRAM_VF_MFGINT"

	if (vars[v,VAR_IGNALL1])
		_flags[_num_flags++] = "BHND_NVRAM_VF_IGNALL1"
		
	if (_num_flags == 0)
		_flags[_num_flags++] = "0"

	return (join(_flags, "|", _num_flags))
}

# emit the bhnd_sprom_offsets for a given variable revision key
function emit_var_sprom_offsets (v, revk)
{
	emit(sprintf("{{%u, %u}, (struct bhnd_sprom_offset[]) {\n",
	    vars[revk,REV_START],
	    vars[revk,REV_END]))
	output_depth++

	num_offs = vars[revk,REV_NUM_OFFS]
	num_offs_written = 0
	elem_count = 0
	for (offset = 0; offset < num_offs; offset++) {
		offk = subkey(revk, OFF, offset"")
		num_segs = vars[offk,OFF_NUM_SEGS]

		for (seg = 0; seg < num_segs; seg++) {
			segk = subkey(offk, OFF_SEG, seg"")

			for (seg_n = 0; seg_n < vars[segk,SEG_COUNT]; seg_n++) {
				seg_addr = vars[segk,SEG_ADDR]
				seg_addr += TSIZE[vars[segk,SEG_TYPE]] * seg_n

				emit(sprintf("{%s, %s, %s, %s, %s},\n",
				    seg_addr,
				    (seg > 0) ? "true" : "false",
				    DTYPE[vars[segk,SEG_TYPE]],
				    vars[segk,SEG_SHIFT],
				    vars[segk,SEG_MASK]))

				num_offs_written++
			}
		}
	}

	output_depth--
	emit("}, " num_offs_written "},\n")
}

# emit a bhnd_nvram_vardef for variable name `v`
function emit_nvram_vardef (v)
{
	emit(sprintf("{\"%s\", %s, %s, %s, (struct bhnd_sprom_vardefn[]) {\n",
		    v suffix,
		    DTYPE[vars[v,VAR_BASE_TYPE]],
		    FMT[vars[v,VAR_FMT]],
		    gen_var_flags(v)))
	output_depth++

	for (rev = 0; rev < vars[v,NUM_REVS]; rev++) {
		revk = subkey(v, REV, rev"")
		emit_var_sprom_offsets(v, revk)
	}

	output_depth--
	emit("}, " vars[v,NUM_REVS] "},\n")
}

# emit a header name #define for variable `v`
function emit_var_namedef (v)
{
	emit("#define\tBHND_NVAR_" toupper(v) "\t\"" v "\"\n")
}

# generate a set of var offset definitions for struct variable `st_vid`
function gen_struct_var_offsets (vid, revk, st_vid, st_revk, base_addr)
{
	# Copy all offsets to the new variable
	for (offset = 0; offset < vars[revk,REV_NUM_OFFS]; offset++) {
		st_offk = subkey(st_revk, OFF, offset"")
		offk = subkey(revk, OFF, offset"")

		# Copy all segments to the new variable, applying base
		# address adjustment
		num_segs = vars[st_offk,OFF_NUM_SEGS]
		vars[offk,OFF_NUM_SEGS] = num_segs

		for (seg = 0; seg < num_segs; seg++) {
			st_segk = subkey(st_offk, OFF_SEG, seg"")
			segk = subkey(offk, OFF_SEG, seg"")

			vars[segk,SEG_ADDR]	= vars[st_segk,SEG_ADDR] + \
			    base_addr""
			vars[segk,SEG_COUNT]	= vars[st_segk,SEG_COUNT]
			vars[segk,SEG_TYPE]	= vars[st_segk,SEG_TYPE]
			vars[segk,SEG_MASK]	= vars[st_segk,SEG_MASK]
			vars[segk,SEG_SHIFT]	= vars[st_segk,SEG_SHIFT]
		}
	}
}

# generate a complete set of variable definitions for struct variable `st_vid`.
function gen_struct_vars (st_vid)
{
	st = vars[st_vid,VAR_STRUCT]
	st_max_off = 0

	# determine the total number of variables to generate
	for (st_rev = 0; st_rev < structs[st,NUM_REVS]; st_rev++) {
		srevk = subkey(st, REV, st_rev"")
		for (off = 0; off < structs[srevk,REV_NUM_OFFS]; off++) {
			if (off > st_max_off)
				st_max_off = off
		}
	}

	# generate variable records for each defined struct offset
	for (off = 0; off <= st_max_off; off++) {
		# Construct basic variable definition
		v = st_vid off""
		vars[v,VAR_TYPE]	= vars[st_vid,VAR_TYPE]
		vars[v,VAR_BASE_TYPE]	= vars[st_vid,VAR_BASE_TYPE]
		vars[v,VAR_FMT]		= vars[st_vid,VAR_FMT]
		vars[v,VAR_PRIVATE]	= vars[st_vid,VAR_PRIVATE]
		vars[v,VAR_ARRAY]	= vars[st_vid,VAR_ARRAY]
		vars[v,VAR_IGNALL1]	= vars[st_vid,VAR_IGNALL1]
		vars[v,NUM_REVS]	= 0

		# Add to output variable list
		output_vars[num_output_vars++] = v

		# Construct revision / offset entries
		for (srev = 0; srev < structs[st,NUM_REVS]; srev++) {
			# Struct revision key
			st_revk = subkey(st, REV, srev"")

			# Skip offsets not defined for this revision
			if (off > structs[st_revk,REV_NUM_OFFS])
				continue

			# Strut offset key and associated base address */
			offk = subkey(st_revk, OFF, off"")
			base_addr = structs[offk,SEG_ADDR]

			for (vrev = 0; vrev < vars[st_vid,NUM_REVS]; vrev++) {
				st_var_revk = subkey(st_vid, REV, vrev"")
				v_start	= vars[st_var_revk,REV_START]
				v_end	= vars[st_var_revk,REV_END]
				s_start	= structs[st_revk,REV_START]
				s_end	= structs[st_revk,REV_END]

				# We don't support computing the union
				# of partially overlapping ranges
				if ((v_start < s_start && v_end >= s_start) ||
				    (v_start <= s_end && v_end > s_end))
				{
					errorx("partially overlapping " \
					    "revision ranges are not supported")
				}

				# skip variables revs that are not within
				# the struct offset's compatibility range
				if (v_start < s_start || v_start > s_end ||
				    v_end < s_start || v_end > s_end)
					continue

				# Generate the new revision record
				rev = vars[v,NUM_REVS] ""
				revk = subkey(v, REV, rev)
				vars[v,NUM_REVS]++

				vars[revk,DEF_LINE]	= vars[st_var_revk,DEF_LINE]
				vars[revk,REV_START]	= v_start
				vars[revk,REV_END]	= v_end
				vars[revk,REV_NUM_OFFS] = \
				    vars[st_var_revk,REV_NUM_OFFS]

				gen_struct_var_offsets(v, revk, st_vid, st_var_revk,
				    base_addr)
			}
		}
	}
}


END {
	# Skip completion handling if exiting from an error
	if (_EARLY_EXIT)
		exit 1

	# Check for complete block closure
	if (!in_parser_context(RootContext)) {
		state = parser_state_get()
		block_start = get(state, _line)
		errorx("missing '}' for block opened on line " block_start "")
	}

	# XXX disabled
	print "Done"
	exit 0

	# Generate concrete variable definitions for all struct variables
	for (v in var_names) {
		if (vars[v,VAR_STRUCT] != null) {
			gen_struct_vars(v)
		} else {
			output_vars[num_output_vars++] = v
		}
	}

	# Apply lexicographical sorting. To support more effecient table
	# searching, we guarantee a stable sort order (using C collation).
	sort(output_vars)

	# Truncate output file and write common header
	printf("") > OUTPUT_FILE
	emit("/*\n")
	emit(" * THIS FILE IS AUTOMATICALLY GENERATED. DO NOT EDIT.\n")
	emit(" *\n")
	emit(" * generated from nvram map: " FILENAME "\n")
	emit(" */\n")
	emit("\n")

	# Emit all variable definitions
	if (OUT_T == OUT_T_DATA) {
		emit("#include <dev/bhnd/nvram/bhnd_nvram_common.h>\n")
		emit("static const struct bhnd_nvram_vardefn "\
		    "bhnd_nvram_vardefs[] = {\n")
		output_depth++
		for (i = 0; i < num_output_vars; i++)
			emit_nvram_vardef(output_vars[i])
		output_depth--
		emit("};\n")
	} else if (OUT_T == OUT_T_HEADER) {
		for (i = 0; i < num_output_vars; i++)
			emit_var_namedef(output_vars[i])
	}

	printf("%u variable records written to %s\n", num_output_vars,
	    OUTPUT_FILE) >> "/dev/stderr"
}


#
# Print usage
#
function usage ()
{
	print "usage: bhnd_nvram_map.awk <input map> [-hd] [-o output file]"
	_EARLY_EXIT = 1
	exit 1
}

#
# Join all array elements with the given separator
#
function join (array, sep, count)
{
	if (count == 0)
		return ("")

	_result = array[0]
	for (_ji = 1; _ji < count; _ji++)
		_result = _result sep array[_ji]

	return (_result)
}

#
# Sort a contiguous integer-indexed array, using standard awk comparison
# operators over its values.
#
function sort (array) {
	# determine array size
	_sort_alen = 0

	for (_ssort_key in array)
		_sort_alen++

	if (_sort_alen <= 1)
		return

	# perform sort
	_qsort(array, 0, _sort_alen-1)
}

function _qsort (array, first, last)
{
	if (first >= last)
		return

	# select pivot element
	_qpivot = int(first + int((last-first+1) * rand()))
	_qleft = first
	_qright = last

	_qpivot_val = array[_qpivot]

	# partition
	while (_qleft <= _qright) {
		while (array[_qleft] < _qpivot_val)
			_qleft++

		while (array[_qright] > _qpivot_val)
			_qright--

		# swap
		if (_qleft <= _qright) {
			_qleft_val = array[_qleft]
			_qright_val = array[_qright]
			
			array[_qleft] = _qright_val
			array[_qright] = _qleft_val

			_qleft++
			_qright--
		}
	}

	# sort the partitions
	_qsort(array, first, _qright)
	_qsort(array, _qleft, last)
}

#
# Print msg to output file, without indentation
#
function emit_ni (msg)
{
	printf("%s", msg) >> OUTPUT_FILE
}

#
# Print msg to output file, indented for the current `output_depth`
#
function emit (msg)
{
	for (_ind = 0; _ind < output_depth; _ind++)
		emit_ni("\t")

	emit_ni(msg)
}

#
# Print a warning to stderr
#
function warn (msg)
{
	print "warning:", msg, "at", FILENAME, "line", NR > "/dev/stderr"
}

#
# Print an warning message without including the source line information
#
function warnx (msg)
{
	print "warning:", msg > "/dev/stderr"
}

#
# Print a compiler error to stderr
#
function error (msg)
{
	errorx(msg " at " FILENAME " line " NR ":\n\t" $0)
}

#
# Print an error message without including the source line information
#
function errorx (msg)
{
	print "error:", msg > "/dev/stderr"
	_EARLY_EXIT=1
	exit 1
}

#
# Print a debug output message
#
function debug (msg)
{
	if (!DEBUG)
		return
	for (_di = 1; _di < depth; _di++)
		printf("\t") > "/dev/stderr"
	print msg > "/dev/stderr"
}

#
# Return an array key composed of the given (parent, selector, child)
# tuple.
# The child argument is optional and may be omitted.
#
function subkey (parent, selector, child)
{
	if (child != null)
		return (parent SUBSEP selector SUBSEP child)
	else
		return (parent SUBSEP selector)
}

#
# Advance to the next non-comment input record
#
function next_line ()
{
	do {
		_result = getline
	} while (_result > 0 && $0 ~ /^[ \t]*#.*/) # skip comment lines
	return (_result)
}

#
# Advance to the next input record and verify that it matches @p regex
#
function getline_matching (regex)
{
	_result = next_line()
	if (_result <= 0)
		return (_result)

	if ($0 ~ regex)
		return (1)

	return (-1)
}

#
# Shift the current fields left by `n`.
#
# If all fields are consumed and the optional do_getline argument is true,
# read the next line.
#
function shiftf (n, do_getline)
{
	if (n > NF) error("shift past end of line")
	for (_si = 1; _si <= NF-n; _si++) {
		$(_si) = $(_si+n)
	}
	NF = NF - n

	if (NF == 0 && do_getline)
		next_line()
}

#
# Parse a revision descriptor from the current line.
#
function parse_revdesc (result)
{
	_rstart = 0
	_rend = 0

	if ($2 ~ "[0-9]*-[0-9*]") {
		split($2, _revrange, "[ \t]*-[ \t]*")
		_rstart = _revrange[1]
		_rend = _revrange[2]
	} else if ($2 ~ "(>|>=|<|<=)" && $3 ~ "[1-9][0-9]*") {
		if ($2 == ">") {
			_rstart = int($3)+1
			_rend = REV_MAX
		} else if ($2 == ">=") {
			_rstart = int($3)
			_rend = REV_MAX
		} else if ($2 == "<" && int($3) > 0) {
			_rstart = 0
			_rend = int($3)-1
		} else if ($2 == "<=") {
			_rstart = 0
			_rend = int($3)-1
		} else {
			error("invalid revision descriptor")
		}
	} else if ($2 ~ "[1-9][0-9]*") {
		_rstart = int($2)
		_rend = int($2)
	} else {
		error("invalid revision descriptor")
	}

	result[REV_START] = _rstart
	result[REV_END] = _rend
}


# Push a new parser state.
function parser_state_push (ctx, is_block, _state) {
	depth++

	_state = obj_new(ParseState)
	set(_state, _ctx, ctx)
	set(_state, _is_block, is_block)
	set(_state, _line, NR)

	lappend(_g_parse_stack, _state)
}

# Fetch the current parser state
function parser_state_get (_top)
{
	if (list_empty(_g_parse_stack))
		errorx("parser_state_get() called with empty parse stack")

	_top = ltail(_g_parse_stack)
	return (lvalue(_top))
}

# Pop the current parser state
function parser_state_pop (_top)
{
	if (list_empty(_g_parse_stack))
		errorx("parser_state_pop() called with empty parse stack")

	_top = ltail(_g_parse_stack)
	ldelete(_g_parse_stack, _top)
}

# Fetch the current context object associated with this parser state
# If class is not null, the object will be asserted as being an instance
# of the class.
function parser_state_get_context (class, _ctx_obj)
{
	_ctx_obj = get(parser_state_get(), _ctx)
	if (class != null)
		obj_assert_class(_ctx_obj, class)

	return (_ctx_obj)
}

#
# Find opening brace and push a new parser state for a brace-delimited block.
#
# The name may be null, in which case the STATE_IDENT variable will not be
# defined in this scope
#
function parser_state_open_block (ctx)
{
	if ($0 ~ "{" || getline_matching("^[ \t]*{") > 0) {
		parser_state_push(ctx, 1)
		sub("^[^{]+{", "", $0)
		return
	}

	error("found '"$1 "' instead of expected '{'")
}

#
# Find closing brace and pop parser states until the first
# brace-delimited block is discarded.
#
function parser_state_close_block (_next_state, _found_block)
{
	if ($0 !~ "}")
		error("internal error - no closing brace")

	# pop states until we exit the first enclosing block
	do {
		_next_state = parser_state_get()
		_found_block = get(_next_state, _is_block)
		parser_state_pop()
	} while (!_found_block)

	# strip everything prior to the block closure
	sub("^[^}]*}", "", $0)
}

# Evaluates to true if the current parser state is defined with a context of
# the given class
function in_parser_context (class, _ctx)
{
	return (obj_is_instanceof(parser_state_get_context(), class))
}

# struct definition
$1 == ST_STRUCT_BLOCK && in_parser_context(RootContext) {
	name = $2

	# Remove array[] specifier
	if (sub(/\[\]$/, "", name) == 0)
		error("expected '" name "[]', not '" name "'")

	if (name !~ IDENT_REGEX || name ~ TYPES_REGEX)
		error("invalid identifier '" name "'")

	# Add top-level struct entry 
	if ((name,DEF_LINE) in structs) 
		error("struct identifier '" name "' previously defined on " \
		    "line " structs[name,DEF_LINE])
	structs[name,DEF_LINE] = NR
	structs[name,NUM_REVS] = 0

	# Open the block 
	debug("struct " name " {")
	open_block(ST_STRUCT_BLOCK, name)
}

# struct srom base offsets
$1 == ST_SROM_DEFN && in_parser_context(Struct) {
	sid = g(STATE_IDENT)

	# parse revision descriptor
	rev_desc[REV_START] = 0
	parse_revdesc(rev_desc)

	# assign revision id
	rev = structs[sid,NUM_REVS] ""
	revk = subkey(sid, REV, rev)
	structs[sid,NUM_REVS]++

	# init basic revision state
	structs[revk,REV_START] = rev_desc[REV_START]
	structs[revk,REV_END] = rev_desc[REV_END]

	if (match($0, "\\[[^]]*\\]") <= 0)
		error("expected base address array")

	addrs_str = substr($0, RSTART+1, RLENGTH-2)
	num_offs = split(addrs_str, addrs, ",[ \t]*")
	structs[revk, REV_NUM_OFFS] = num_offs
	for (i = 1; i <= num_offs; i++) {
		offk = subkey(revk, OFF, (i-1) "")

		if (addrs[i] !~ HEX_REGEX)
			error("invalid base address '" addrs[i] "'")

		structs[offk,SEG_ADDR] = addrs[i]
	}

	debug("struct_srom " structs[revk,REV_START] "... [" addrs_str "]")
	next
}

# close any previous srom revision descriptor
$1 == ST_SROM_DEFN && in_parser_context(SROM) {
	pop_state()
}

# variable srom revision descriptor
$1 == ST_SROM_DEFN && in_parser_context(Var) {
	# parse revision descriptor
	parse_revdesc(rev_desc)

	# assign revision id
	vid = g(STATE_IDENT)
	rev = vars[vid,NUM_REVS] ""
	revk = subkey(vid, REV, rev)
	vars[vid,NUM_REVS]++

	# vend scoped rev/revk variables for use in the
	# revision offset block
	push("rev_id", rev)
	push("rev_key", revk)

	# init basic revision state
	vars[revk,DEF_LINE] = NR
	vars[revk,REV_START] = rev_desc[REV_START]
	vars[revk,REV_END] = rev_desc[REV_END]
	vars[revk,REV_NUM_OFFS] = 0

	debug("srom " rev_desc[REV_START] "-" rev_desc[REV_END] " {")
	push_state(ST_SROM_DEFN, null, null, 0)

	# seek to the first offset definition
	do {
		shiftf(1)
	} while ($1 !~ SROM_OFF_REGEX && NF > 0)
}

#
# Parse an offset declaration from the current line.
#
function parse_offset_segment (revk, offk)
{
	vid = g(STATE_IDENT)

	# use explicit type if specified, otherwise use the variable's
	# common type
	if ($1 !~ HEX_REGEX) {
		type = $1
		if (type !~ TYPES_REGEX)
			error("unknown field type '" type "'")

		shiftf(1)
	} else {
		type = vars[vid,VAR_TYPE]
	}

	# read offset value
	offset = $1
	if (offset !~ HEX_REGEX)
		error("invalid offset value '" offset "'")

	# extract byte count[], base type, and width
	if (match(type, ARRAY_REGEX"$") > 0) {
		count = int(substr(type, RSTART+1, RLENGTH-2))
		type = substr(type, 1, RSTART-1)
	} else {
		count = 1
	}
	width = TSIZE[type]

	# seek to attributes or end of the offset expr
	sub("^[^,(|){}]+", "", $0)

	# parse attributes
	mask=TMASK[type]
	shift=0

	if ($1 ~ "^\\(") {
		# extract attribute list
		if (match($0, "\\([^|\(\)]*\\)") <= 0)
			error("expected attribute list")
		attr_str = substr($0, RSTART+1, RLENGTH-2)

		# drop from input line
		$0 = substr($0, RSTART+RLENGTH, length($0) - RSTART+RLENGTH)

		# parse attributes
		num_attr = split(attr_str, attrs, ",[ \t]*")
		for (i = 1; i <= num_attr; i++) {
			attr = attrs[i]
			if (sub("^&[ \t]*", "", attr) > 0) {
				mask = attr
			} else if (sub("^<<[ \t]*", "", attr) > 0) {
				shift = "-"attr
			} else if (sub("^>>[ \t]*", "", attr) > 0) {
				shift = attr
			} else {
				error("unknown attribute '" attr "'")
			}
		}
	}

	# assign segment id
	seg = vars[offk,OFF_NUM_SEGS] ""
	segk = subkey(offk, OFF_SEG, seg)
	vars[offk,OFF_NUM_SEGS]++

	vars[segk,SEG_ADDR]	= offset + (width * _oi)
	vars[segk,SEG_COUNT]	= count
	vars[segk,SEG_TYPE]	= type
	vars[segk,SEG_MASK]	= mask
	vars[segk,SEG_SHIFT]	= shift

	debug("{"vars[segk,SEG_ADDR]", "type", "mask", "shift"}" \
		_comma)
}

# revision offset definition
$1 ~ SROM_OFF_REGEX && in_parser_context(SROM) {
	vid = g(STATE_IDENT)

	# fetch rev id/key defined by our parent block
	rev = g("rev_id")
	revk = g("rev_key")

	# parse all offsets
	do {
		# assign offset id
		off = vars[revk,REV_NUM_OFFS] ""
		offk = subkey(revk, OFF, off)
		vars[revk,REV_NUM_OFFS]++

		# initialize segment count
		vars[offk,DEF_LINE] = NR
		vars[offk,OFF_NUM_SEGS] = 0

		debug("[")
		# parse all segments
		do {
			parse_offset_segment(revk, offk)
			_more_seg = ($1 == "|")
			if (_more_seg)
				shiftf(1, 1)
		} while (_more_seg)
		debug("],")
		_more_vals = ($1 == ",")
		if (_more_vals)
			shiftf(1, 1)
	} while (_more_vals)
}

# Return the type constant for `name`, if any
function type_named (name, _n, _type)
{
	for (_n = lhead(BaseTypes); _n != null; _n = lnext(_n)) {
		_type = lvalue(_n)
		if (get(_type, _name) == name)
			return (_type)
	}

	return (null)	
}

# Search `vars` list for a variable with `name`
function var_list_search (vars, name, _n, _val)
{
	for (_n = lhead(vars); _n != null; _n = lnext(_n)) {
		_val = lvalue(_n)
		if (get(_val, _name) == name)
			return (_val)
	}

	return (null)
}

#
# Parse a type string to either the Type, ArrayType, or null if
# the type is not recognized.
#
function parse_type_string (str, _base, _count)
{
	if (match(str, ARRAY_REGEX"$") > 0) {
		# Extract count and base type
		_count = substr(str, RSTART+1, RLENGTH-2)
		sub(ARRAY_REGEX"$", "", str)

		# Look for base type
		if ((_base = type_named(str)) == null)
			return (null)

		return (array_type_new(_base, _count))
	} else {
		return (type_named(str))
	}
}

# variable definition
(($1 == "private" && $2 ~ TYPES_REGEX) || $1 ~ TYPES_REGEX) &&
    in_parser_context(SymbolContext) \
{
	ctx = parser_state_get_context(SymbolContext)

	# Check for 'private' flag
	var_flags = list_new()
	if ($1 == "private") {
		lappend(var_flags, VarPrivate)
		private = 1
		shiftf(1)
	}

	# Parse the type string
	if ((type = parse_type_string($1)) == null)
		error("unknown type '" $1 "'")

	# Construct new variable instance
	name = $2
	offsets = list_new()
	var = var_new(name, type, type_get_default_format(type), var_flags,
	    offsets, NR)
	debug((private ? "private " : "") type_desc(type) " " name " {")

	# Add to our parent context
	var_list = get(ctx, _vars)
	if ((prev_var = var_list_search(var_list, name)) != null) {
		error("variable identifier '" name "' previously defined on " \
		    "line " get(prev_var, _line))
	}
	lappend(var_list, var)

	# Push our variable definition block
	parser_state_open_block(var)
}

# variable parameters
$1 ~ IDENT_REGEX && $2 ~ IDENT_REGEX && in_state(ST_VAR_BLOCK) {
	vid = g(STATE_IDENT)
	if ($1 == PROP_T_SFMT) {
		if (!$2 in FMT)
			error("invalid fmt '" $2 "'")

		vars[vid,VAR_FMT] = $2
		debug($1 "=" FMT[$2])
	} else if ($1 == PROP_T_ALL1 && $2 == "ignore") {
		vars[vid,VAR_IGNALL1] = 1
	} else {
		error("unknown parameter " $1)
	}
	next
}

# Skip comments and blank lines
/^[ \t]*#/ || /^$/ {
	next
}

# Close blocks
/}/ && !in_parser_context(RootContext) {
	while (!in_parser_context(RootContext) && $0 ~ "}") {
		parser_state_close_block();
		debug("}")
	}
	next
}

# Report unbalanced '}'
/}/ && in_parser_context(RootContext) {
	error("extra '}'")
}

# Invalid variable type
$1 && in_parser_context(SymbolContext) {
	error("unknown type '" $1 "'")
}

# Generic parse failure
{
	error("unrecognized statement")
}
