#pragma once



/// Meta flags can be used like this:
/// transfer.Transfer (someVar, "varname", kHideInEditorMask);
/// The GenerateTypeTreeTransfer for example reads the metaflag mask and stores it in the TypeTree
typedef enum
{
	kNoTransferFlags = 0,
	/// Putting this mask in a transfer will make the variable be hidden in the property editor
	kHideInEditorMask = 1 << 0,

	/// Makes a variable not editable in the property editor
	kNotEditableMask = 1 << 4,

	/// There are 3 types of PPtrs: kStrongPPtrMask, default (weak pointer)
	/// a Strong PPtr forces the referenced object to be cloned.
	/// A Weak PPtr doesnt clone the referenced object, but if the referenced object is being cloned anyway (eg. If another (strong) pptr references this object)
	/// this PPtr will be remapped to the cloned object
	/// If an  object  referenced by a WeakPPtr is not cloned, it will stay the same when duplicating and cloning, but be NULLed when templating
	kStrongPPtrMask = 1 << 6,
	// unused  = 1 << 7,

	/// kTreatIntegerValueAsBoolean makes an integer variable appear as a checkbox in the editor, be written as true/false to JSON, etc
	kTreatIntegerValueAsBoolean = 1 << 8,

	// unused = 1 << 9,
	// unused = 1 << 10,
	// unused = 1 << 11,

	/// When the options of a serializer tells you to serialize debug properties kSerializeDebugProperties
	/// All debug properties have to be marked kDebugPropertyMask
	/// Debug properties are shown in expert mode in the inspector but are not serialized normally
	kDebugPropertyMask = 1 << 12,

	// Used in TypeTree to indicate that a property is aligned to a 4-byte boundary. Do not specify this flag when
	// transferring variables; call transfer.Align() instead.
	kAlignBytesFlag = 1 << 14,

	// Used in TypeTree to indicate that some child of this typetree node uses kAlignBytesFlag. Do not use this flag.
	kAnyChildUsesAlignBytesFlag = 1 << 15,

	// unused = 1 << 16,
	// unused = 1 << 18,

	// Ignore this property when reading or writing .meta files
	kIgnoreInMetaFiles = 1 << 19,

	// When reading meta files and this property is not present, read array entry name instead (for backwards compatibility).
	kTransferAsArrayEntryNameInMetaFiles = 1 << 20,

	// When writing YAML Files, uses the flow mapping style (all properties in one line, with "{}").
	kTransferUsingFlowMappingStyle = 1 << 21,

	// Tells SerializedProperty to generate bitwise difference information for this field.
	kGenerateBitwiseDifferences = 1 << 22,

	// Makes a variable not be exposed to the animation system
	kDontAnimate = 1 << 23,

	// Encodes a 64-bit signed or unsigned integer as a hex string in text serializers.
	kTransferHex64 = 1 << 24,

	// Use to differentiate between uint16 and C# Char.
	kCharPropertyMask = 1 << 25,

	//do not check if string is utf8 valid, (usually all string must be valid utf string, but sometimes we serialize pure binary data to string,
	//for example TextAsset files with extension .bytes. In this case this validation should be turned off)
	//Player builds will never validate data. In editor we validate correct encoding of strings by default.
	kDontValidateUTF8 = 1 << 26,

	// Fixed buffers are serialized as arrays, use this flag to differentiate between regular arrays and fixed buffers.
	kFixedBufferFlag = 1 << 27
} TransferMetaFlags;