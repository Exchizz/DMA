#include "mailbox.h"
#include <sys/ioctl.h>
#include <array>
#include <cstdint>
#include <cstdio>

int main (int /*argc*/, char * /*argv*/ [])
{
	auto ret_val = 0;

	// Prepares the communication buffer
	// Ref: https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface

	// static to make sure the alignment is right even if > std::max_align_t
	alignas (16) static auto p = std::array <uint32_t, 32> {};

	p [0] = 7 * sizeof (p [0]); // size in bytes
	p [1] = 0x00000000; // process request

	p [2] = 0x00060001; // tag: Get DMA channels
	p [3] = 4;          // buffer size in bytes
	p [4] = 0x00000000; // request/response code
	p [5] = 0;          // value buffer, padded to 32 bits

	p [6] = 0;          // end of the tag list

	// Mailbox property
	auto fd = mbox_open ();
	if (fd < 0)
	{
		fprintf (stderr, "Cannot open the mailbox\n");
		ret_val = -1;
	}
	else
	{
		const auto ret_ioctl = ioctl (fd, IOCTL_MBOX_PROPERTY, p.data ());
		if (ret_ioctl < 0)
		{
			fprintf (stderr, "Failed to querry mailbox property\n");
			ret_val = -1;
		}
	}
	if (fd >= 0)
	{
		mbox_close (fd);
	}

	// Checks the response
	if (ret_val == 0)
	{
		if (p [1] != 0x80000000)
		{
			fprintf (stderr, "mailbox returned an error code: 0x%08X\n", p [1]);
			ret_val = -1;
		}
		else if ((p [4] & 0x80000000) == 0 || (p [4] & 0x7FFFFFFF) < 4)
		{
			fprintf (stderr, "mailbox returned an unexpected response code: 0x%08X\n", p [4]);
			ret_val = -1;
		}
	}
	if (ret_val == 0)
	{
		const auto val = p [5];
		printf ("Raw value: 0x%08X. Available DMA channels:\n", val);
		auto found_flag = bool { false };
		for (int k = 0; k < 16; ++k)
		{
			if ((val & (1 << k)) != 0)
			{
				printf ("%s%d", found_flag ? ", " : "", k);
				found_flag = true;
			}
		}
		printf ("%s\n", found_flag ? "" : "None");
	}

	return ret_val;
}
