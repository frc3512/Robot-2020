#!/usr/bin/env python3
"""Called by the build system."""

import argparse
import os
import re
import shutil


class FileWriter:

    def __init__(self, filename):
        """Opens a temporary file with the given name.

        It will replace the original file, if one exists, only if the temporary
        file's contents differ from the original upon closing. Otherwise, the
        temporary file will be discarded.
        """
        self.filename = filename

    def __enter__(self):
        self.file = open(self.filename + ".tmp", "w")
        return self.file

    def __exit__(self, type, value, traceback):
        self.file.close()

        try:
            with open(self.filename) as file:
                old_contents = file.read()
        except FileNotFoundError:
            shutil.move(self.filename + ".tmp", self.filename)
            return

        with open(self.filename + ".tmp") as file:
            new_contents = file.read()

        if old_contents != new_contents:
            shutil.move(self.filename + ".tmp", self.filename)
        else:
            os.remove(self.filename + ".tmp")


def write_msg_header(output_dir, msg_name, member_var_types,
                     constructor_arg_types, member_var_names):
    """Write message header file.

    Keyword arguments:
    output_dir -- output directory root for source
    msg_name -- name of message in camel case
    member_var_types -- list of member variable types
    constructor_arg_types -- list of function argument types
    member_var_names -- list of member variable names
    """
    filename = "python/Packet.hpp.in"
    with open(filename) as input:
        content = input.read()

    content = content.replace("${CLASS_NAME}", f"{msg_name}Packet")

    variables = [
        f"    int8_t ID = static_cast<int8_t>(PacketType::k{msg_name});"
    ]
    default_vals = {"double": " = 0.0;", "int": " = 0;", "bool": " = false;"}
    for i in range(len(member_var_types)):
        variable = f"    {member_var_types[i]} {member_var_names[i]}"
        try:
            variable += default_vals[member_var_types[i]]
        except KeyError:
            # No known default value, so use default constructor instead
            variable += ";"
        variables.append(variable)
    content = content.replace("${VARIABLES}", "\n".join(variables))

    ctor = f"{msg_name}Packet("
    ctor += ", ".join([
        x[0] + " " + x[1] for x in zip(constructor_arg_types, member_var_names)
    ])
    ctor += ");"

    content = content.replace("${CTOR}", ctor)

    filename = f"{output_dir}/include/communications/{msg_name}Packet.hpp"
    with FileWriter(filename) as output:
        output.write(content)


def write_msg_source(output_dir, msg_name, constructor_arg_types,
                     member_var_names, serial_names):
    """Write message source file.

    Keyword arguments:
    output_dir -- output directory root for source
    msg_name -- name of message in camel case
    constructor_arg_types -- list of function argument types
    member_var_names -- list of member variable names
    serial_names -- list of member variable names to serialize/deserialize
    """
    filename = "python/Packet.cpp.in"
    with open(filename) as input:
        content = input.read()

    content = content.replace("${CLASS_NAME}", f"{msg_name}Packet")

    ctor_args = ", ".join([
        x[0] + " " + x[1] for x in zip(constructor_arg_types, member_var_names)
    ])
    content = content.replace("${CTOR_ARGS}", ctor_args)

    member_init = [f"    this->{name} = {name};" for name in member_var_names]
    content = content.replace("${CTOR_INIT}", "\n".join(member_init))

    serial = [f"    packet << {name};" for name in serial_names]
    content = content.replace("${SERIAL}", "\n".join(serial))

    deserial = [f"    packet >> {name};" for name in serial_names]
    content = content.replace("${DESERIAL}", "\n".join(deserial))

    filename = f"{output_dir}/cpp/communications/{msg_name}Packet.cpp"
    with FileWriter(filename) as output:
        output.write(content)


def write_packettype_header(output_dir, msg_names):
    """Write PacketType.hpp header file.

    Keyword arguments:
    output_dir -- output directory root for source
    msg_names -- list of packet message names
    """
    filename = "python/PacketType.hpp.in"
    with open(filename) as input:
        content = input.read()

    enum_type = "enum class PacketType : int8_t"
    types = ["k" + x for x in msg_names]
    singleline_types = ", ".join(types)

    len_first_line = len(enum_type) + len(singleline_types) + len(" {  };")
    if len_first_line <= 80:
        content = content.replace("${ENUM_VALUES}", f" {singleline_types} ")
    else:
        multiline_types = ",".join(["\n    " + x for x in types])
        content = content.replace("${ENUM_VALUES}", multiline_types + "\n")

    filename = f"{output_dir}/include/communications/PacketType.hpp"
    with FileWriter(filename) as output:
        output.write(content)


def write_publishnodebase_header(output_dir, msg_names):
    """Write PublishNodeBase.hpp header file.

    Keyword arguments:
    output_dir -- output directory root for source
    msg_names -- list of packet message names
    """
    filename = "python/PublishNodeBase.hpp.in"
    with open(filename) as input:
        content = input.read()

    # Generate includes
    includes = [
        f'#include "communications/{msg_name}Packet.hpp"'
        for msg_name in msg_names
    ]
    content = content.replace("${INCLUDES}", "\n".join(includes))

    # Generate function prototypes
    functions = [
        f"    virtual void ProcessMessage(const {msg_name}Packet& message) {{}}"
        for msg_name in msg_names
    ]
    content = content.replace("${FUNCTIONS}", "\n".join(functions))

    filename = f"{output_dir}/include/communications/PublishNodeBase.hpp"
    with FileWriter(filename) as output:
        output.write(content)


def write_publishnodebase_source(output_dir, msg_names):
    """Write PublishNodeBase.cpp source file.

    Keyword arguments:
    output_dir -- output directory root for source
    msg_names -- list of packet message names
    """
    filename = "python/PublishNodeBase.cpp.in"
    with open(filename) as input:
        content = input.read()

    case_blocks = []
    for i, msg_name in enumerate(msg_names):
        case_blocks.append(f"    case PacketType::k{msg_name}:")
        case_blocks.append(
            f"        DeserializeImpl<{msg_name}Packet>(message);")
        case_blocks.append(f"        break;")
    content = content.replace("${DESERIAL_AND_PROC}", "\n".join(case_blocks))

    filename = f"{output_dir}/cpp/communications/PublishNodeBase.cpp"
    with FileWriter(filename) as output:
        output.write(content)


def main():
    parser = argparse.ArgumentParser(
        description=
        "Parses message descriptor files from the given directory and generates C++ source for serializing and deserializing them."
    )
    parser.add_argument("--input", help="directory containing message files")
    parser.add_argument("--output",
                        help="directory to which to write C++ source")
    args = parser.parse_args()

    msg_files = [
        os.path.join(dp, f)
        for dp, dn, fn in os.walk(args.input)
        for f in fn
        if f.endswith(".msg")
    ]

    # Make destination folders for messages
    if not os.path.exists(f"{args.output}/cpp/communications"):
        os.makedirs(f"{args.output}/cpp/communications")
    if not os.path.exists(f"{args.output}/include/communications"):
        os.makedirs(f"{args.output}/include/communications")

    # Parse schema files
    var_regex = re.compile(r"(?P<type>[\w:]+)\s+(?P<name>\w+)")
    for filename in msg_files:
        with open(filename, "r") as msgfile:
            member_var_types = ["std::string"]
            member_var_names = ["topic"]
            constructor_arg_types = ["wpi::StringRef"]
            serial_names = ["ID", "topic"]
            for line in msgfile:
                # Strip comments
                if line.find("#") != -1:
                    line = line[:line.find("#")]

                match = var_regex.search(line)
                if match:
                    type = match.group("type")
                    name = match.group("name")
                    if type == "string":
                        member_var_types.append("std::string")
                        constructor_arg_types.append("wpi::StringRef")
                    else:
                        member_var_types.append(type)
                        constructor_arg_types.append(type)
                    member_var_names.append(name)
                    serial_names.append(name)

            msg_name = os.path.splitext(os.path.basename(filename))[0]
            write_msg_header(
                args.output,
                msg_name,
                member_var_types,
                constructor_arg_types,
                member_var_names,
            )
            write_msg_source(
                args.output,
                msg_name,
                constructor_arg_types,
                member_var_names,
                serial_names,
            )
    msg_names = [
        os.path.splitext(os.path.basename(name))[0] for name in msg_files
    ]
    msg_names = sorted(msg_names)
    write_packettype_header(args.output, msg_names)
    write_publishnodebase_header(args.output, msg_names)
    write_publishnodebase_source(args.output, msg_names)


if __name__ == "__main__":
    main()
