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

	OUTPUT_FILE = null

	# Seed rand()
	srand()

	# Output type
	OUT_T = null
	OUT_T_HEADER = "HEADER"
	OUT_T_DATA = "DATA"

	# Tab width to use when calculating output alignment
	TAB_WIDTH = 8

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

	# Common Regexs
	UINT_REGEX	= "^(0|[1-9][0-9]*),?$"
	HEX_REGEX	= "^0x[A-Fa-f0-9]+,?$"

	ARRAY_REGEX	= "\\[(0|[1-9][0-9]*)\\]"
	TYPES_REGEX	= "^(((u|i)(8|16|32))|char)("ARRAY_REGEX")?,?$"

	IDENT_REGEX	= "^[A-Za-z_][A-Za-z0-9_]*,?$"
	SROM_OFF_REGEX	= "("TYPES_REGEX"|"HEX_REGEX")"

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
		class_add_prop(List, _count, "count")

	ListNode = class_new("ListNode")
		class_add_prop(ListNode, _v, "v")
		class_add_prop(ListNode, _next, "next")
		class_add_prop(ListNode, _prev, "prev")

	# Preprocessor Constant
	MacroDefine = class_new("MacroDefine")
		class_add_prop(MacroDefine, _name, "name")
		class_add_prop(MacroDefine, _value, "value")

	# ParseState definition
	ParseState = class_new("ParseState")
		class_add_prop(ParseState, _ctx, "ctx")
		class_add_prop(ParseState, _is_block, "is_block")
		class_add_prop(ParseState, _line, "line")

	# Variable property type
	PropType = class_new("PropType")
		class_add_prop(PropType, _name, "name")
		class_add_prop(PropType, _const, "const")

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

	# AST node class
	AST = class_new("AST")
		class_add_prop(AST, _line, "line")

	SymbolContext = class_new("SymbolContext", AST)
		class_add_prop(SymbolContext, _vars, "vars")

	# Root symbol context
	RootContext = class_new("RootContext", SymbolContext)
		class_add_prop(RootContext, _structs, "structs")

	RevDesc = class_new("RevDesc", AST)
		class_add_prop(RevDesc, _start, "start")
		class_add_prop(RevDesc, _end, "end")

	Offset = class_new("Offset", AST)
		class_add_prop(Offset, _segments, "segments")

	SROM = class_new("SROM", AST)
		class_add_prop(SROM, _revisions, "revisions")
		class_add_prop(SROM, _offsets, "offsets")
		class_add_prop(SROM, _offset_base, "offset_base")

	Segment = class_new("Segment", AST)
		class_add_prop(Segment, _offset, "offset")
		class_add_prop(Segment, _type, "type")
		class_add_prop(Segment, _mask, "mask")
		class_add_prop(Segment, _shift, "shift")

	VarProp = class_new("VarProp", AST)
		class_add_prop(VarProp, _prop_type, "name")
		class_add_prop(VarProp, _value, "value")

	StringConstant = class_new("StringConstant", AST)
		class_add_prop(StringConstant, _value, "value")		# string
		class_add_prop(StringConstant, _continued, "continued")	# bool

	Var = class_new("Var", AST)
		class_add_prop(Var, _name, "name")	# string
		class_add_prop(Var, _desc, "desc")	# StringConstant
		class_add_prop(Var, _help, "help")	# StringConstant
		class_add_prop(Var, _type, "type")	# AbstractType
		class_add_prop(Var, _fmt, "fmt")	# SFmt
		class_add_prop(Var, _props, "props")	# List<VarProp>
		class_add_prop(Var, _sroms, "sroms")	# List<SROM>

	# Provides a RevDesc, and a list of base offsets at which the
	# struct is defined
	StructSROM = class_new("StructSROM", AST)
		class_add_prop(StructSROM, _revisions, "revisions")
		class_add_prop(StructSROM, _offsets, "offsets")

	Struct = class_new("Struct", SymbolContext)
		class_add_prop(Struct, _name, "name")
		class_add_prop(Struct, _sroms, "sroms")

	# Format Constants
	SFmtHex		= sfmt_new("hex", "BHND_NVRAM_SFMT_HEX")
	SFmtDec 	= sfmt_new("decimal", "BHND_NVRAM_SFMT_DEC")
	SFmtCCODE	= sfmt_new("ccode", "BHND_NVRAM_SFMT_CCODE")
	SFmtMAC		= sfmt_new("macaddr", "BHND_NVRAM_SFMT_MACADDR")
	SFmtLEDDC	= sfmt_new("led_dc", "BHND_NVRAM_SFMT_LEDDC")

	# Data Type Constants
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

	# Property type constants
	PropTypePrivate	= var_prop_type_new("private", "BHND_NVRAM_VF_MFGINT")
	PropTypeIgnAll1	= var_prop_type_new("ignall1", "BHND_NVRAM_VF_IGNALL1")

	StringFormats = list_new()
		lappend(StringFormats, SFmtHex)
		lappend(StringFormats, SFmtDec)
		lappend(StringFormats, SFmtCCODE)
		lappend(StringFormats, SFmtMAC)
		lappend(StringFormats, SFmtLEDDC)

	# Create the parse state stack
	_g_parse_stack_depth = 0
	_g_parse_stack[0] = null

	# Push the root parse state
	_g_root_ctx = obj_new(RootContext)
	set(_g_root_ctx, _vars, list_new())
	set(_g_root_ctx, _structs, list_new())

	parser_state_push(_g_root_ctx, 0)
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

	# Fetch all of our parsed records
	all_vars = get(_g_root_ctx, _vars)
	structs = get(_g_root_ctx, _structs)

	# Generate concrete variable definitions for all struct variables
	for (_st = lhead(structs); _st != null; _st = lnext(_st)) {
		struct = lvalue(_st)
		v = get(struct, _vars)

		for (_vn = lhead(v); _vn != null; _vn = lnext(_vn)) {			
			gen_struct_vars(struct, lvalue(_vn), all_vars)
		}
	}

	# Build a map of all variables
	num_output_vars = list_size(all_vars)
	for (_n = lhead(all_vars); _n != null; _n = lnext(_n)) {
		var = lvalue(_n)
		name = get(var, _name)

		if (name in var_names) {
			errorc(get(var, _line), "duplicate variable " \
			    "identifier '" name "' previously defined at " \
			    "line " get(var_names[name], _line))
		}

		var_names[name] = var
	}

	# Apply lexicographical sorting to our variable names. To support more
	# effecient table searching, we guarantee a stable sort order (using C
	# collation).
	i = 0
	for (name in var_names)
		output_vars[i++] = name

	sort(output_vars, num_output_vars)

	# Replace the variable names in the sort array with the actual
	# variable instances
	for (i = 0; i < num_output_vars; i++)
		output_vars[i] = var_names[output_vars[i]]

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
		# Produce our array of #defines
		def_count = 0
		max_namelen = 0
		for (i = 0; i < num_output_vars; i++) {
			var = output_vars[i]
			vname = get(var, _name)
			cname = var_get_constant_name(var)

			# Variable name
			m_name = cname
			m = macro_new(cname, "\"" vname "\"")

			max_namelen = max(max_namelen, length(m_name))
			output_defs[def_count++] = m

			# Variable array length
			if (var_has_array_type(var)) {
				m_name = cname "_MAXLEN"
				m = macro_new(m_name, var_get_count(var))

				output_defs[def_count++] = m
				max_namelen = max(max_namelen, length(m_name))
			}
		}

		# Calculate value tab alignment position for our defines
		tab_align = max_namelen
		tab_align += (TAB_WIDTH - (tab_align % TAB_WIDTH)) % TAB_WIDTH
		tab_align /= TAB_WIDTH

		for (i = 0; i < def_count; i++) {
			emit_macro_define(output_defs[i], tab_align)
		}
	}

	printf("%u variable records written to %s\n", num_output_vars,
	    OUTPUT_FILE) >> "/dev/stderr"
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

# Return true if the given property property ID is defined on class
function class_has_prop_id (class, prop_id, _super)
{
	if (_super != null)
		errorx("class_has_prop_id() must be called with two arguments")

	if (class == null)
		return (false)

	# Check class<->prop cache
	if ((class, prop_id) in _g_class_prop_cache)
		return (1)

	# Otherwise, slow path
	if (!obj_is_class(class))
		errorx(class " is not a class object")

	if (_super != null)
		errorx("class_has_prop_id() must be called with two arguments")

	if (!obj_is_class(class))
		errorx(class " is not a class object")

	for (_super = class; _super != null; _super = obj_get_class(_super)) {
		if (!((_super,CLS_PROP,prop_id) in _g_obj))
			continue

		# Found; add to class<->prop cache
		_g_class_prop_cache[class,prop_id] = 1
		return (1)
	}

	return (0)
}

# Return true if the given property prop is defined on class
function class_has_property (class, prop)
{
	if (!(PROP_ID in prop))
		return (false)

	return (class_has_prop_id(class, prop[PROP_ID]))
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

# Return the property ID for a given class-defined property
function class_get_prop_id (class, prop)
{
	if (class == null)
		errorx("class_get_prop_id() on null class")

	if (!class_has_property(class, prop)) {
		errorx("requested undefined property '" prop[PROP_NAME] "on " \
		    class_get_name(class))
	}

	return (prop[PROP_ID])
}

# Return the property ID for a given class-defined property name
function class_get_named_prop_id (class, name, _prop_id)
{
	if (class == null)
		errorx("class_get_prop_id() on null class")

	if (!(name in _g_prop_names))
		errorx("requested undefined property '" name "'")

	_prop_id = _g_prop_names[name]

	if (!class_has_prop_id(class, _prop_id)) {
		errorx("requested undefined property '" _g_props[_prop_id] \
		    "' on " class_get_name(class))
	}

	return (_prop_id)
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

function obj_dump (obj)
{
	# TODO: dump properties too?
	print(class_get_name(obj_get_class(obj)) "<" obj ">")
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
		errorx(class_get_name(obj_get_class(obj)) "<" obj "> is not " \
		    "an instance of " class_get_name(class))
	}
}

# Return an abstract property ID for a given property
function obj_get_prop_id (obj, prop)
{
	if (obj == null)
		errorx("obj_get_prop_id() on null object")

	return (class_get_prop_id(obj_get_class(obj), prop))
}


# Return the property ID for a given property name
function obj_get_named_prop_id (obj, name)
{
	if (obj == null)
		errorx("obj_get_named_prop_id() on null object")

	return (class_get_named_prop_id(obj_get_class(obj), name))
}

# Set a property on obj
function set (obj, prop, value, _class)
{
	return (prop_set(obj, prop[PROP_ID], value))
}

# Get a property value defined on obj
function get (obj, prop, _class)
{
	return (prop_get(obj, prop[PROP_ID]))
}

# Set a property on obj, using a property ID returned by obj_get_prop_id() or
# class_get_prop_id()
function prop_set (obj, prop_id, value, _class)
{
	if (obj == null) {
		errorx("setting property '" _g_props[prop_id] \
		    "' on null object")
	}

	_class = obj_get_class(obj)
	if (_class == null)
		errorx(obj " has no superclass")

	if (!class_has_prop_id(_class, prop_id)) {
		errorx("requested undefined property '" _g_props[prop_id] \
		    "' (" prop_id ") on " class_get_name(_class))
	}

	_g_obj[obj,OBJ_PROP,prop_id] = value
}

# Fetch a value property value from obj, using a property ID returned by
# obj_get_prop_id() or class_get_prop_id()
function prop_get (obj, prop_id, _class)
{
	if (obj == null) {
		errorx("requested property '" _g_props[prop_id] \
		    "' on null object")
	}

	_class = obj_get_class(obj)
	if (_class == null)
		errorx(obj " has no superclass")

	if (!class_has_prop_id(_class, prop_id)) {
		errorx("requested undefined property '" _g_props[prop_id] \
		    "' (" prop_id ") on " class_get_name(_class))
	}

	return (_g_obj[obj,OBJ_PROP,prop_id])
}

# Create a new MacroDefine instance
function macro_new (name, value, _obj)
{
	_obj = obj_new(MacroDefine)
	set(_obj, _name, name)
	set(_obj, _value, value)

	return (_obj)
}

# Create an empty list
function list_new (_obj)
{
	_obj = obj_new(List)
	set(_obj, _count, 0)

	return (_obj)
}

# Return true if the list is empty
function list_empty (list)
{
	return (lhead(list) == null)
}

# Return number of elements in the list
function list_size (list)
{
	obj_assert_class(list, List)
	return (get(list, _count))
}

# Return the nth list value, or throw an error if the element does not exist.
#
# This is a O(n) operation.
function list_at (list, n)
{
	return (lvalue(list_node_at(list, n)))
}

# Return the nth list node, or throw an error if the element does not exist.
#
# This is a O(n) operation.
function list_node_at (list, n, _node, _pos)
{
	if (n >= list_size(list)) {
		errorx("requested element " n " in " list_size(list) \
		    " element list")
		list_dump(list)
	}

	_pos = 0
	for (_node = lhead(list); _node != null; _node = lnext(_node)) {
		if (_pos == n)
			return (_node)

		_pos++
	}

	errorx("unreachable!")
}

function list_dump (list, _node)
{
	obj_assert_class(list, List)
	print "List<" list "> (nelem=" get(list, _count) ")"
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

	# Bump the list count
	set(list, _count, get(list, _count) + 1)
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

	# Decrement the list count
	set(list, _count, get(list, _count) - 1)
}

# Return the value associated with the given list node
function lvalue (lnode)
{
	obj_assert_class(lnode, ListNode)
	return (get(lnode, _v))
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

# Return the base type for a given type instance.
function type_base (type)
{
	if (obj_is_instanceof(type, ArrayType))
		return (type_base(get(type, _type)))

	obj_assert_class(type, Type)
	return (type)
}

# Return a string representation of the given type
function type_to_string (type, _base_type)
{
	if (obj_is_instanceof(type, ArrayType)) {
		_base_type = type_base(type)
		return (type_to_string(_base_type) "[" get(type, _count) "]")
	}
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

		return (array_type_new(_base, int(_count)))
	} else {
		return (type_named(str))
	}
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

# Create a new SFmt instance
function sfmt_new (name, constant, _obj)
{
	_obj = obj_new(SFmt)
	set(_obj, _name, name)
	set(_obj, _const, constant)

	return (_obj)
}


# Return the SFmt constant for `name`, if any
function sfmt_named (name, _n, _sfmt)
{
	for (_n = lhead(StringFormats); _n != null; _n = lnext(_n)) {
		_sfmt = lvalue(_n)
		if (get(_sfmt, _name) == name)
			return (_sfmt)
	}

	return (null)
}

# Create a new PropType instance
function var_prop_type_new (name, constant, _obj)
{
	_obj = obj_new(PropType)
	set(_obj, _name, name)
	set(_obj, _const, constant)

	return (_obj)
}

# Create a new VarProp instance
function var_prop_new (prop_type, value, _obj)
{
	_obj = obj_new(VarProp)
	set(_obj, _prop_type, prop_type)
	set(_obj, _value, value)
	set(_obj, _line, NR)

	return (_obj)
}


# Search `props` list for a property type `prop_type`
function var_props_list_search (props, prop_type, _n, _prop, _t)
{
	for (_n = lhead(props); _n != null; _n = lnext(_n)) {
		_prop = lvalue(_n)
		_t = get(_prop, _prop_type)

		if (get(_t, _name) == get(prop_type, _name))
			return (_prop)
	}

	return (null)
}

# Create a new StringConstant AST node
function stringconstant_new (value, continued, _obj)
{
	_obj = obj_new(StringConstant)
	set(_obj, _value, value)
	set(_obj, _continued, continued)
	set(_obj, _line, NR)

	return (_obj)
}

# Create an empty StringConstant AST node to which additional lines
# may be appended
function stringconstant_empty (_obj)
{
	return (stringconstant_new("", 1))
}

# Parse an input string and return a new string constant
# instance
function stringconstant_parse_line (line, _obj)
{
	_obj = stringconstant_empty()
	stringconstant_append_line(_obj, line)
	return (_obj)
}

# Parse and apend an additional line to this string constant
function stringconstant_append_line (str, line, _cont, _strbuf)
{
	obj_assert_class(str, StringConstant)

	# Must be marked for continuation
	if (!get(str, _continued)) {
		errorx("can't append to non-continuation string '" \
		    get(str, _value) "'")
	}

	# Look for (and remove) any line continuation
	_cont = sub(/\\[ \t]*$/, "", line)

	# Trim leading and trailing whitespace
	sub(/(^[ \t]+|[ \t]+$)/, "", line)

	# Append to existing buffer
	if ((_strbuf = get(str, _value)) == NULL)
		set(str, _value, line)
	else
		set(str, _value, _strbuf " " line)

	# Update line continuation setting
	set(str, _continued, _cont)
}

# Create a new Var instance
function var_new (name, type, fmt, props, _obj)
{
	_obj = obj_new(Var)
	set(_obj, _name, name)
	set(_obj, _type, type)
	set(_obj, _fmt, fmt)
	set(_obj, _props, props)
	set(_obj, _sroms, list_new())
	set(_obj, _line, NR)

	return (_obj)
}

# Return true if `var` has an array type
function var_has_array_type (var, _vtype)
{
	obj_assert_class(var, Var)
	_vtype = get(var, _type)
	return (obj_is_instanceof(_vtype, ArrayType))
}

# Return the number of array elements defined by this variable's type,
# or 1 if the variable does not have an array type.
function var_get_count (var, _vtype)
{
	obj_assert_class(var, Var)

	if (!var_has_array_type(var))
		return (1)

	_vtype = get(var, _type)
	return (get(_vtype, _count))
}

# Return the base preprocessor constant name to be used with `var`
function var_get_constant_name (var, _vname)
{
	obj_assert_class(var, Var)

	_vname = get(var, _name)
	return("BHND_NVAR_" toupper(_vname))
}

# Create a new Struct instance
function struct_new (name, _obj)
{
	_obj = obj_new(Struct)
	set(_obj, _name, name)
	set(_obj, _vars, list_new())
	set(_obj, _sroms, list_new())
	set(_obj, _line, NR)

	return (_obj)
}

# Create a new struct SROM base offset descriptor instance
function struct_srom_new (rev_desc, _obj)
{
	_obj = obj_new(StructSROM)
	set(_obj, _revisions, rev_desc)
	set(_obj, _offsets, list_new())
	set(_obj, _line, NR)

	return (_obj)
}

# Create a new SROM instance
function srom_new (rev_desc, _obj)
{
	_obj = obj_new(SROM)
	set(_obj, _revisions, rev_desc)
	set(_obj, _offsets, list_new())
	set(_obj, _offset_base, 0)
	set(_obj, _line, NR)

	return (_obj)
}
	
# Create a new Offset instance
function offset_new (_obj)
{
	_obj = obj_new(Offset)
	set(_obj, _segments, list_new())
	set(_obj, _line, NR)

	return (_obj)
}

# Create a new Segment instance
function segment_new (offset, type, mask, shift, _obj)
{
	_obj = obj_new(Segment)
	set(_obj, _offset, offset)
	set(_obj, _type, type)
	set(_obj, _mask, mask)
	set(_obj, _shift, shift)
	set(_obj, _line, NR)

	return (_obj)
}

# Return a human-readable representation of a Segment instance
function segment_to_string (seg, _str, _t, _attr_count)
{
	# include type (if specified)
	_t = get(seg, _type)
	if (_t != null)
		_str = (type_to_string(_t) " ")

	# include offset
	_str = (_str get(seg, _offset))

	# append list of attributes
	_attr_count = 0
	if (_m != null)
		_g_attr_list[_attr_count++] = ("&" _m)
	if (_s != null)
		_g_attr_list[_attr_count++] = _s

	if (_attr_count == 0)
		return (_str)

	return (_str " (" join(_g_attr_list, ", ", _attr_count) ")")
}

# return the flag definition for variable `v`
function gen_var_flags (v, _vtype, _vprops, _vprop, _vprop_type, _flags,
    _num_flags)
{
	_num_flags = 0;
	_vtype = get(v, _type)
	_vprops = get(v, _props)

	for (_np = lhead(_vprops); _np != null; _np = lnext(_np)) {
		_vprop = lvalue(_np)
		_vprop_type = get(_vprop, _prop_type)
		_flags[_num_flags++] = get(_vprop_type, _const)
	}

	if (obj_is_instanceof(_vtype, ArrayType))
		_flags[_num_flags++] = "BHND_NVRAM_VF_ARRAY"

	if (_num_flags == 0)
		_flags[_num_flags++] = "0"

	return (join(_flags, "|", _num_flags))
}

# emit the bhnd_sprom_offsets for a given variable SROM record
function emit_var_sprom_offsets (v, srom)
{
	var_type = get(v, _type)
	revs = get(srom, _revisions)
	offs = get(srom, _offsets)

	# Fetch base address
	offset_base = get(srom, _offset_base)	

	emit(sprintf("{{%u, %u}, (struct bhnd_sprom_offset[]) {\n",
	    get(revs, _start),
	    get(revs, _end)))

	output_depth++

	segments_written = 0
	for (_on = lhead(offs); _on != null; _on = lnext(_on)) {
		off = lvalue(_on)
		segs = get(off, _segments)

		seg_pos = 0
		for (_sn = lhead(segs); _sn != null; _sn = lnext(_sn)) {
			seg = lvalue(_sn)

			# Fetch segment type; if not set, fall back on the
			# variable type
			if ((seg_type = get(seg, _type)) == null)
				seg_type = var_type

			seg_base_type = type_base(seg_type)

			seg_off = get(seg, _offset)
			seg_width = get(seg_base_type, _width)

			if ((seg_shift = get(seg, _shift)) == null)
				seg_shift = 0

			if ((seg_mask = get(seg, _mask)) == null)
				seg_mask = get(seg_base_type, _mask)

			# Determine array count
			if (obj_is_instanceof(seg_type, ArrayType))
				seg_count = get(seg_type, _count)
			else
				seg_count = 1
	
			for (seg_n = 0; seg_n < seg_count; seg_n++) {
				seg_n_off = (offset_base + seg_off + \
				    (seg_n * seg_width))
	
				emit(sprintf("{%s, %s, %s, %s, %s},\n",
				    seg_n_off,
				    (seg_pos > 0) ? "true" : "false",
				    get(seg_base_type, _const),
				    seg_shift,
				    seg_mask))
	
				segments_written++
			}

			seg_pos++
		}
	}

	output_depth--
	emit("}, " segments_written "},\n")
}

# emit a bhnd_nvram_vardef for variable name `v`
function emit_nvram_vardef (v)
{
	type = type_base(get(v, _type))
	sroms = get(v, _sroms)

	emit(sprintf("{\"%s\", %s, %s, %s, (struct bhnd_sprom_vardefn[]) {\n",
	    get(v, _name),
	    get(type, _const),
	    get(get(v, _fmt), _const),
	    gen_var_flags(v)))
	output_depth++

	for (_nr = lhead(sroms); _nr != null; _nr = lnext(_nr)) {
		srom = lvalue(_nr)
		emit_var_sprom_offsets(v, srom)
	}

	output_depth--
	emit("}, " list_size(get(v, _sroms)) "},\n")
}

# emit a MacroDefine constant, aligning the value to `tab_align`
function emit_macro_define (macro, tab_align, _pad, _tabstr)
{
	# Determine required padding to reach the desired alignment
	_pad = int(length(get(macro, _name)) / TAB_WIDTH)
	if (tab_align > _pad)
		_pad = tab_align - _pad
	else
		_pad = 0

	for (_pi = 0; _pi <= _pad; _pi++)
		_tabstr = _tabstr "\t"

	emit("#define\t" get(macro, _name) _tabstr get(macro, _value) "\n")
}

# generate a complete set of variable definitions for struct `st` and
# struct variable `v`, appending them to list `results`
function gen_struct_vars (st, var, results)
{
	sroms = get(st, _sroms)

	# Determine the number of variables to generate
	max_off_count = 0
	for (_n = lhead(sroms); _n != null; _n = lnext(_n)) {
		srom = lvalue(_n)
		offs = get(srom, _offsets)

		if (list_size(offs) > max_off_count)
			max_off_count = list_size(offs)
	}

	# Generate variable records for each defined struct offset
	for (off = 0; off < max_off_count; off++) {
		# Construct basic variable definition		
		v = var_new( \
		    get(var, _name) "" off,
		    get(var, _type),
		    get(var, _fmt),
		    get(var, _props))

		# Preserve original line number
		set(v, _line, get(var, _line))

		# Add to output variable list
		lappend(results, v)

		# Iterate over the struct's SROM entries and merge
		# matching struct SROM base addresses with corresponding
		# variable SROM entries.
		for (_n = lhead(sroms); _n != null; _n = lnext(_n)) {
			# Fetch struct SROM base offsets
			s_srom = lvalue(_n)
			s_revs = get(s_srom, _revisions)
			s_start = get(s_revs, _start)
			s_end = get(s_revs, _end)
			s_base_offs = get(s_srom, _offsets)

			# Skip offsets not defined for this revision
			if (off >= list_size(s_base_offs))
				continue

			# Actual base address
			s_base_addr = list_at(s_base_offs, off)

			# Now iterate over struct variable's SROM entries
			# looking for matches
			v_sroms = get(var, _sroms)
			for (_vs = lhead(v_sroms); _vs != null;
			    _vs = lnext(_vs))
			{
				v_srom = lvalue(_vs)
				v_revs = get(v_srom, _revisions)
				v_start	= get(v_revs, _start)
				v_end	= get(v_revs, _end)

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

				# Copy SROM record to the new variable, setting
				# a new base addr
				n_sroms = get(v, _sroms)
				n_srom = srom_new(v_revs)

				set(n_srom, _offsets, get(v_srom, _offsets))
				set(n_srom, _offset_base, s_base_addr)

				# Preserve original line number
				set(n_srom, _line, get(v_srom, _line))

				lappend(n_sroms, n_srom)
			}
		}
	}
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
# Return the maximum of two values
#
function max (lhs, rhs)
{
	return (lhs > rhs ? lhs : rhs)
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
function sort (array, array_len, _key) {
	# determine array size
	if (array_len == null) {
		array_len = 0

		for (_key in array)
			array_len++
	}

	if (array_len <= 1)
		return

	# perform sort
	_qsort(array, 0, array_len-1)
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
# Print a compiler error to stderr with a caller supplied
# line number
#
function errorc (line, msg)
{
	errorx(msg " at " FILENAME " line " line)
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
	for (_di = 1; _di < _g_parse_stack_depth; _di++)
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

# Push a new parser state.
function parser_state_push (ctx, is_block, _state) {
	_state = obj_new(ParseState)
	set(_state, _ctx, ctx)
	set(_state, _is_block, is_block)
	set(_state, _line, NR)

	_g_parse_stack_depth++
	_g_parse_stack[_g_parse_stack_depth] = _state
}

# Fetch the current parser state
function parser_state_get (_top)
{
	if (_g_parse_stack_depth == 0)
		errorx("parser_state_get() called with empty parse stack")

	return (_g_parse_stack[_g_parse_stack_depth])
}

# Pop the current parser state
function parser_state_pop (_top, _block_state)
{
	if (_g_parse_stack_depth == 0)
		errorx("parser_state_pop() called with empty parse stack")

	_closes_block = get(parser_state_get(), _is_block)

	delete _g_parse_stack[_g_parse_stack_depth]
	_g_parse_stack_depth--

	if (_closes_block)
		debug("}")
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

#
# Parse and return a revision descriptor from the current line.
#
function parse_revdesc (_rstart, _rend, _robj)
{
	_rstart = 0
	_rend = 0

	if ($2 ~ "[0-9]*-[0-9*]") {
		split($2, _g_rev_range, "[ \t]*-[ \t]*")
		_rstart = _g_rev_range[1]
		_rend = _g_rev_range[2]
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

	_robj = obj_new(RevDesc)
	set(_robj, _start, _rstart)
	set(_robj, _end, _rend)
	set(_robj, _line, NR)

	return (_robj)
}

# close any previously opened srom revision descriptor
$1 == "srom" && in_parser_context(SROM) {
	parser_state_pop()
}

# struct definition
$1 == "struct" && in_parser_context(RootContext) {
	ctx = parser_state_get_context(RootContext)
	structs = get(ctx, _structs)

	name = $2

	# Remove array[] specifier
	if (sub(/\[\]$/, "", name) == 0)
		error("expected '" name "[]', not '" name "'")

	if (name !~ IDENT_REGEX || name ~ TYPES_REGEX)
		error("invalid identifier '" name "'")

	# Add struct entry
	struct = struct_new(name)
	lappend(structs, struct)

	# Open the block 
	debug("struct " name "[] {")
	parser_state_open_block(struct)
}

# struct srom base offsets
$1 == "srom" && in_parser_context(Struct) {
	ctx = parser_state_get_context(Struct)
	sroms = get(ctx, _sroms)

	# Parse revision descriptor
	revs = parse_revdesc()

	# Register the SROM record with our struct
	srom = struct_srom_new(revs)
	lappend(sroms, srom)

	offsets = get(srom, _offsets)

	# Parse the offset array
	if (match($0, "\\[[^]]*\\]") <= 0)
		error("expected base address array")
	addrs_str = substr($0, RSTART+1, RLENGTH-2)
	num_offs = split(addrs_str, addrs, ",[ \t]*")

	for (i = 1; i <= num_offs; i++) {
		if (addrs[i] !~ HEX_REGEX)
			error("invalid base address '" addrs[i] "'")
		lappend(offsets, addrs[i])
	}

	debug("srom " get(revs,_start) "-" get(revs,_end) " [" addrs_str "]")
	next
}

#
# Parse an offset segment declaration from the current line
#
function parse_segment (_type, _offset, _attrs, _num_attr, _attr, _mask, _shift,
    _off_desc)
{
	if (_type != null) {
		errorx("parse_offset_segment() must be called with no " \
		    "arguments")
	}

	# use explicit type if specified
	if ($1 !~ HEX_REGEX) {
		if ((_type = parse_type_string($1)) == null)
			error("unknown type '" $1 "'")

		shiftf(1)
	}

	# read offset value
	_offset = $1
	if (_offset !~ HEX_REGEX && _offset !~ UINT_REGEX)
		error("invalid offset value '" _offset "'")
	sub(",$", "", _offset)	# drop trailing comma, if any

	# seek to offset (attributes...) or end of the offset expr (|,)
	sub("^[^,(|){}]+", "", $0)

	# parse attributes
	if ($1 ~ "^\\(") {
		# extract attribute list
		if (match($0, "\\([^|\(\)]*\\)") <= 0)
			error("expected attribute list")

		_attrs = substr($0, RSTART+1, RLENGTH-2)

		# drop attribute list from the input line
		$0 = substr($0, RSTART+RLENGTH, length($0) - RSTART+RLENGTH)

		# parse attributes
		_num_attr = split(_attrs, _g_attrs, ",[ \t]*")
		for (i = 1; i <= _num_attr; i++) {
			_attr = _g_attrs[i]
	
			if (sub("^&[ \t]*", "", _attr) > 0) {
				_mask = _attr
			} else if (sub("^<<[ \t]*", "", _attr) > 0) {
				_shift = "-"_attr
			} else if (sub("^>>[ \t]*", "", _attr) > 0) {
				_shift = _attr
			} else {
				error("unknown attribute '" _attr "'")
			}
		}
	}

	return (segment_new(_offset, _type, _mask, _shift))
}

# variable definition block
(($1 == "private" && $2 ~ TYPES_REGEX) || $1 ~ TYPES_REGEX) &&
    in_parser_context(SymbolContext) \
{
	ctx = parser_state_get_context(SymbolContext)

	# Check for 'private' flag
	props = list_new()
	if ($1 == "private") {
		lappend(props, var_prop_new(PropTypePrivate, 1))
		private = 1
		shiftf(1)
	}

	# Parse the type string
	if ((type = parse_type_string($1)) == null)
		error("unknown type '" $1 "'")

	# Construct new variable instance
	name = $2
	fmt = get(type_base(type), _default_fmt)
	var = var_new(name, type, fmt, props)
	debug((private ? "private " : "") type_to_string(type) " " name " {")

	# Add to our parent context
	var_list = get(ctx, _vars)
	lappend(var_list, var)

	# Push our variable definition block
	parser_state_open_block(var)
}

# variable parameters
$1 ~ IDENT_REGEX && $2 ~ IDENT_REGEX && in_parser_context(Var) {
	var = parser_state_get_context(Var)
	props = get(var, _props)

	if ($1 == "sfmt") {
		debug($1 " " $2)

		if ((sfmt = sfmt_named($2)) == null)
			error("invalid fmt '" $2 "'")

		set(var, _fmt, sfmt)
	} else if ($1 == "all1") {
		debug($1 " " $2)

		# Must not have already been set
		if ((dup = var_props_list_search(props, PropTypeIgnAll1))) {
			error("all1 previously specified on line " \
			    get(dup, _line))
		}

		# Check argument
		if ($2 != "ignore") {
			error("unknown all1 value '" $2 "', expected 'ignore'")
		}

		# Add to variable's property list
		lappend(props, var_prop_new(PropTypeIgnAll1, 1))
	} else if ($1 == "desc" || $1 == "help") {
		# Fetch an indirect property reference for either the 'desc'
		# or 'help' property
		prop_id = obj_get_named_prop_id(var, $1)

		# Check for an existing definition
		if ((str = prop_get(var, prop_id)) != null) {
			error(get(var, _name) " '" $1 "' redefined " \
			    "(previously defined on line " get(str, _line) ")")
		}

		# Seek to the start of the help/desc string
		shiftf(1)

		# Parse the first line
		str = stringconstant_parse_line($0)

		# Incrementally parse line continuations
		while (get(str, _continued)) {
			getline
			stringconstant_append_line(str, $0)
		}

		debug("desc \"" get(str, _value) "\"")

		# Add to the var object
		prop_set(var, prop_id, str)
	} else {
		error("unknown parameter " $1)
	}

	next
}

# Variable SROM descriptor
$1 == "srom" && in_parser_context(Var) {
	var = parser_state_get_context(Var)
	sroms = get(var, _sroms)

	# Parse revision descriptor and register SROM
	# instance
	revs = parse_revdesc()
	srom = srom_new(revs)
	lappend(sroms, srom)

	debug("srom " get(revs, _start) "-" get(revs, _end))

	# Push new SROM parser state
	parser_state_push(srom, 0)

	# seek to the first offset definition
	do {
		shiftf(1)
	} while ($1 !~ SROM_OFF_REGEX && NF > 0)
}

# SROM revision offset definition
$1 ~ SROM_OFF_REGEX && in_parser_context(SROM) {
	srom = parser_state_get_context(SROM)
	offsets = get(srom, _offsets)

	# parse all offsets
	do {
		offset = obj_new(Offset)
		segs = list_new()
		set(offset, _segments, segs)

		# parse all segments
		do {
			seg = parse_segment()
			lappend(segs, seg)

			# Do more segments follow?
			more_seg = ($1 == "|")
			if (more_seg)
				shiftf(1, 1)

			if (more_seg)
				debug(segment_to_string(seg) " |")
			else
				debug(segment_to_string(seg))
		} while (more_seg)

		# Do more offsets follow?
		more_vals = ($1 == ",")
		if (more_vals)
			shiftf(1, 1)

		# Add to var's offset list
		lappend(offsets, offset)
	} while (more_vals)
}

# Skip comments and blank lines
/^[ \t]*#/ || /^$/ {
	next
}

# Close blocks
/}/ && !in_parser_context(RootContext) {
	while (!in_parser_context(RootContext) && $0 ~ "}") {
		parser_state_close_block();
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
