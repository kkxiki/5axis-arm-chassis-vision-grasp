#!/usr/bin/env python3
import pathlib, re, sys
p = pathlib.Path(sys.argv[1])
s = p.read_text(encoding="utf-8", errors="surrogateescape")
close_block = """            if (client_socket_ != -1)
            {
#ifndef WIN32
                (void)close(client_socket_);
#else
                (void)closesocket(client_socket_);
#endif
                client_socket_ = -1;
            }"""
if "(void)close(client_socket_);" in s[s.find("~CPPClient"):s.find("template<typename RES>")]:
    sys.exit(0)
begin = s.find("~CPPClient")
end = s.find("template<typename RES>", begin)
if begin < 0 or end < 0:
    raise SystemExit("CPPClient destructor not found; refusing unsafe patch")
section = s[begin:end]
# The vendor file mixes tabs/spaces. Insert after recv_thread_.join() instead
# of depending on the surrounding indentation/braces.
match = re.search(r"recv_thread_\.join\(\);", section)
if not match:
    raise SystemExit("cpp_rpc.hpp destructor layout changed; refusing unsafe patch")
at = begin + match.end()
s = s[:at] + "\n" + close_block + s[at:]
p.write_text(s, encoding="utf-8", errors="surrogateescape")
