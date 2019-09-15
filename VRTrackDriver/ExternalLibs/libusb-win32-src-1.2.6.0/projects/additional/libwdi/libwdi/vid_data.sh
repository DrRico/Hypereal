#!/bin/sh

echo This file recreates vid_data.c according http://www.linux-usb.org/usb.ids
echo

# check that wget and sed are available
type -P wget &>/dev/null || { echo "wget command not found. Aborting." >&2; exit 1; }
type -P sed &>/dev/null || { echo "sed command not found. Aborting." >&2; exit 1; }

# Download the latest version (overwrite previous if newer)
wget -m -nd http://www.linux-usb.org/usb.ids

# Create the sed command file
cat > cmd.sed <<\_EOF
# Header, part 1
s�^#.*List of USB.*�/*\
 * USB vendors, by VID\
 * based on http://www.linux-usb.org/usb.ids by Stephen J. Gowdy\
 *\
 *\
 * This library is free software; you can redistribute it and/or\
 * modify it under the terms of the GNU Lesser General Public\
 * License as published by the Free Software Foundation; either\
 * version 2.1 of the License, or (at your option) any later version.\
 *\
 * This library is distributed in the hope that it will be useful,\
 * but WITHOUT ANY WARRANTY; without even the implied warranty of\
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU\
 * Lesser General Public License for more details.\
 *\
 * You should have received a copy of the GNU Lesser General Public\
 * License along with this library; if not, write to the Free Software\
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA\
 */\
\
#include <stdlib.h>\
#include "libwdi.h"\
\
struct vendor_name \{\
	unsigned short vid;\
	const char* name;\
\};\
\
/*\
 * http://www.linux-usb.org/usb.ids�p

# Header part 2, version
s|# Version:| * Version:|
/ \* Version:/p

# Header Part 3
s|^#.*Date:.*| \*/\
struct vendor_name usb_vendor[] = \{|p

# Footer
$a\
\};\
\
const char* LIBWDI_API wdi_get_vendor_name(unsigned short vid)\
\{\
	int i;\
\
	for(i=0; i<sizeof(usb_vendor)/sizeof(usb_vendor[0]); i++) \{\
		if (usb_vendor[i].vid == vid) \{\
			return usb_vendor[i].name;\
		\}\
	\}\
	return NULL;\
\}

# Main Data
/^[0-9a-fA-F][0-9a-fA-F][0-9a-fA-F][0-9a-fA-F]  /!d
s/^\([0-9a-fA-F][0-9a-fA-F][0-9a-fA-F][0-9a-fA-F]\)  \(.*\)/\	{ 0x\1, "\2" \},/
s/???/?/
s/??/?/
_EOF

# Run sed to generate the source.
sed -f cmd.sed usb.ids > vid_data.c
rm cmd.sed
echo Done.