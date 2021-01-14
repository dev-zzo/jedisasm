"""
JEDEC disassembler for "classic" PAL and GAL devices.
"""

import os.path
import sys

# JEDEC format parsing
# Ref: http://www.pldtool.com/pdf/jesd3c_jedecfmt.pdf

def jedec_commands(fp):
    """
    Generator function to return JEDEC commands contained in a file.
    The file must be opened as a text file.
    """
    delim = fp.read(1)
    while delim != "*":
        if not delim:
            return
        delim = fp.read(1)
    while True:
        buffer = ""
        ch = fp.read(1)
        while ch != "*":
            if ch == "\x03":
                return
            if ch in ("\r", "\n"):
                ch = " "
            buffer += ch
            ch = fp.read(1)
        yield buffer.lstrip()

def load_jedec(fp):
    """
    Process the JEDEC file, returning the complete fuse map as an array of ints
    The file must be opened as a text file.
    """
    fuse_list = []
    num_fuses = None
    default = 0
    for cmd in jedec_commands(fp):
        #print(cmd)
        if cmd.startswith("QF"):
            num_fuses = int(cmd[2:], 10)
        elif cmd.startswith("F"):
            default = int(cmd[1:], 10)
        elif cmd.startswith("L"):
            fuse_list.append(cmd[1:])
    fusemap = [default for _ in range(num_fuses)]
    for c in fuse_list:
        pos, data = c.split(maxsplit=1)
        data = data.replace(" ", "")
        pos = int(pos, 10)
        for fuse, state in zip(range(pos, pos+len(data)), data):
            fusemap[fuse] = int(state)
    return fusemap

def read_jedec(path):
    with open(path, "r") as fp:
        return load_jedec(fp)

class ANDList(list):
    pass
class ORList(list):
    pass

def inverted(x):
    if x[0] == "!":
        return x[1:]
    return "!" + x

def is_term_nonzero(fusemap, indices):
    # Any pair programmed = always zero = ignored
    for i in range(indices[0], indices[1], 2):
        if fusemap[i] == 0 and fusemap[i+1] == 0:
            return False
    return True

def collect_and_terms(fusemap, indices):
    return ANDList([i for i in range(indices[1] - indices[0]) if fusemap[indices[0]+i] == 0])

def append_pair(name_list, name, inv):
    if inv:
        name_list.append(inverted(name))
        name_list.append(name)
    else:
        name_list.append(name)
        name_list.append(inverted(name))

class PALBase:
    def __init__(self, fusemap, pinmap):
        self.fusemap = fusemap
        if pinmap is not None:
            self.pinmap = pinmap
        else:
            self.pinmap = ["PIN%02d" % x for x in range(1, 21)]

    def get_pin_name(self, pin):
        return self.pinmap[pin-1]

    def get_input_names(self):
        """
        Return an array of names for each input (column) of the AND array
        """
        names = []
        for pin, inv in self.get_input_map():
            name = self.get_pin_name(pin)
            if inv:
                names.append(inverted(name))
                names.append(name)
            else:
                names.append(name)
                names.append(inverted(name))
        return names
        
    def get_macrocell_cmt(self, config):
        return []
        
class P16L8(PALBase):
    """
    PAL16L8
    """
    pin_count = 20

    class OLMC:
        def __init__(self, pin, oe_fuses, terms_fuses):
            self.pin = pin
            self.oe_fuses = oe_fuses
            self.terms_fuses = terms_fuses

    macrocell_config = [
        OLMC(19, (   0,   32), [(s, s+32) for s in range(  32,  256, 32)]),
        OLMC(18, ( 256,  288), [(s, s+32) for s in range( 288,  512, 32)]),
        OLMC(17, ( 512,  544), [(s, s+32) for s in range( 544,  768, 32)]),
        OLMC(16, ( 768,  800), [(s, s+32) for s in range( 800, 1024, 32)]),
        OLMC(15, (1024, 1056), [(s, s+32) for s in range(1056, 1280, 32)]),
        OLMC(14, (1280, 1312), [(s, s+32) for s in range(1312, 1536, 32)]),
        OLMC(13, (1536, 1568), [(s, s+32) for s in range(1568, 1792, 32)]),
        OLMC(12, (1792, 1824), [(s, s+32) for s in range(1824, 2048, 32)]),
    ]

    def get_input_map(self):
        return [
            ( 2, False),
            ( 1, False),
            ( 3, False),
            (18, self.is_out_inverted(18)),
            ( 4, False),
            (17, self.is_out_inverted(17)),
            ( 5, False),
            (16, self.is_out_inverted(16)),
            ( 6, False),
            (15, self.is_out_inverted(15)),
            ( 7, False),
            (14, self.is_out_inverted(14)),
            ( 8, False),
            (13, self.is_out_inverted(13)),
            ( 9, False),
            (11, False)]

    def is_out_inverted(self, pin):
        for config in self.macrocell_config:
            if config.pin == pin:
                break
        else:
            return False
        return is_term_nonzero(self.fusemap, config.oe_fuses)

    def get_macrocell_eqns(self, config):
        eqns = []
        name = self.get_pin_name(config.pin)
        if is_term_nonzero(self.fusemap, config.oe_fuses):
            oe_terms = collect_and_terms(self.fusemap, config.oe_fuses)
            if oe_terms:
                eqns.append((name+".oe", oe_terms))
        d_terms = ORList()
        for term_fuses in config.terms_fuses:
            if is_term_nonzero(self.fusemap, term_fuses):
                terms = collect_and_terms(self.fusemap, term_fuses)
                if terms:
                    d_terms.append(terms)
        if d_terms:
            eqns.append((name, d_terms))
        return eqns
        
class G16V8xx(PALBase):
    """
    GAL16V8, common stuff
    """
    pin_count = 20

    class OLMC:
        def __init__(self, pin, oe_fuses, terms_fuses, ac1_fuse, xor_fuse):
            self.pin = pin
            self.oe_fuses = oe_fuses
            self.terms_fuses = terms_fuses
            self.ac1_fuse = ac1_fuse
            self.xor_fuse = xor_fuse

    macrocell_config = [
        OLMC(19, (   0,   32), [(s, s+32) for s in range(  32,  256, 32)], 2120, 2048),
        OLMC(18, ( 256,  288), [(s, s+32) for s in range( 288,  512, 32)], 2121, 2049),
        OLMC(17, ( 512,  544), [(s, s+32) for s in range( 544,  768, 32)], 2122, 2050),
        OLMC(16, ( 768,  800), [(s, s+32) for s in range( 800, 1024, 32)], 2123, 2051),
        OLMC(15, (1024, 1056), [(s, s+32) for s in range(1056, 1280, 32)], 2124, 2052),
        OLMC(14, (1280, 1312), [(s, s+32) for s in range(1312, 1536, 32)], 2125, 2053),
        OLMC(13, (1536, 1568), [(s, s+32) for s in range(1568, 1792, 32)], 2126, 2054),
        OLMC(12, (1792, 1824), [(s, s+32) for s in range(1824, 2048, 32)], 2127, 2055),
    ]

    def get_macrocell_cmt(self, config):
        comments = []
        comments.append("Pin %d; AC1:%d; XOR:%d" % (config.pin, self.fusemap[config.ac1_fuse], self.fusemap[config.xor_fuse]))
        return comments

class G16V8MA(G16V8xx):
    """
    GAL16V8 in Complex configuration
    """
    def __init__(self, fusemap, pinmap):
        G16V8xx.__init__(self, fusemap, pinmap)
        if fusemap[2192] != 1 or fusemap[2193] != 1:
            raise TypeError("incorrect device chosen")

    def get_input_map(self):
        return [
            ( 2, False),
            ( 1, False),
            ( 3, False),
            (18, self.is_out_inverted(18)),
            ( 4, False),
            (17, self.is_out_inverted(17)),
            ( 5, False),
            (16, self.is_out_inverted(16)),
            ( 6, False),
            (15, self.is_out_inverted(15)),
            ( 7, False),
            (14, self.is_out_inverted(14)),
            ( 8, False),
            (13, self.is_out_inverted(13)),
            ( 9, False),
            (11, False)]

    def is_out_inverted(self, pin):
        for config in self.macrocell_config:
            if config.pin == pin:
                break
        else:
            return False
        return is_term_nonzero(self.fusemap, config.oe_fuses) and not self.fusemap[config.xor_fuse]
        
    def get_macrocell_eqns(self, config):
        eqns = []
        name = self.get_pin_name(config.pin)
        if is_term_nonzero(self.fusemap, config.oe_fuses):
            oe_terms = collect_and_terms(self.fusemap, config.oe_fuses)
            if oe_terms:
                eqns.append((name+".oe", oe_terms))
        d_terms = ORList()
        for term_fuses in config.terms_fuses:
            if is_term_nonzero(self.fusemap, term_fuses):
                terms = collect_and_terms(self.fusemap, term_fuses)
                if terms:
                    d_terms.append(terms)
        if d_terms:
            eqns.append((name, d_terms))
        return eqns
        
class G16V8MS(G16V8xx):
    """
    GAL16V8 in Registered configuration
    """
    def __init__(self, fusemap, pinmap):
        G16V8xx.__init__(self, fusemap, pinmap)
        if fusemap[2192] != 0 or fusemap[2193] != 1:
            raise TypeError("incorrect device chosen")

    def get_input_map(self):
        return [
            ( 2, False),
            (19, self.is_out_inverted(19)),
            ( 3, False),
            (18, self.is_out_inverted(18)),
            ( 4, False),
            (17, self.is_out_inverted(17)),
            ( 5, False),
            (16, self.is_out_inverted(16)),
            ( 6, False),
            (15, self.is_out_inverted(15)),
            ( 7, False),
            (14, self.is_out_inverted(14)),
            ( 8, False),
            (13, self.is_out_inverted(13)),
            ( 9, False),
            (12, self.is_out_inverted(12))]

    def is_out_inverted(self, pin):
        for config in self.macrocell_config:
            if config.pin == pin:
                break
        else:
            return False
        return not self.fusemap[config.ac1_fuse] and not self.fusemap[config.xor_fuse]
        
    def get_macrocell_eqns(self, config):
        eqns = []
        name = self.get_pin_name(config.pin)
        if is_term_nonzero(self.fusemap, config.oe_fuses):
            oe_terms = collect_and_terms(self.fusemap, config.oe_fuses)
        else:
            oe_terms = ANDList()
        d_terms = ORList()
        for term_fuses in config.terms_fuses:
            if is_term_nonzero(self.fusemap, term_fuses):
                terms = collect_and_terms(self.fusemap, term_fuses)
                if terms:
                    d_terms.append(terms)
                    
        if self.fusemap[config.ac1_fuse]:
            if oe_terms:
                eqns.append((name+".oe", oe_terms))
            if d_terms:
                eqns.append((name, d_terms))
        else:
            if oe_terms:
                d_terms.insert(0, oe_terms)
            if d_terms:
                eqns.append((name+".d", d_terms))
        return eqns
        
class G16V8AS(G16V8xx):
    """
    GAL16V8 in Simple configuration
    """
    def __init__(self, fusemap, pinmap):
        G16V8xx.__init__(self, fusemap, pinmap)
        if fusemap[2192] != 1 or fusemap[2193] != 0:
            raise TypeError("incorrect device chosen")

    def get_input_map(self):
        return [
            ( 2, False),
            ( 1, False),
            ( 3, False),
            (19, self.is_out_inverted(19)),
            ( 4, False),
            (18, self.is_out_inverted(18)),
            ( 5, False),
            (17, self.is_out_inverted(17)),
            ( 6, False),
            (14, self.is_out_inverted(14)),
            ( 7, False),
            (13, self.is_out_inverted(13)),
            ( 8, False),
            (12, self.is_out_inverted(12)),
            ( 9, False),
            (11, False)]

    def is_out_inverted(self, pin):
        for config in self.macrocell_config:
            if config.pin == pin:
                break
        else:
            return False
        return not self.fusemap[config.ac1_fuse] and not self.fusemap[config.xor_fuse]
        
    def get_macrocell_eqns(self, config):
        eqns = []
        if not self.fusemap[config.ac1_fuse]:
            name = self.get_pin_name(config.pin)
            d_terms = ORList()
            if is_term_nonzero(self.fusemap, config.oe_fuses):
                terms = collect_and_terms(self.fusemap, config.oe_fuses)
                if terms:
                    d_terms.append(terms)
            for term_fuses in config.terms_fuses:
                if is_term_nonzero(self.fusemap, term_fuses):
                    terms = collect_and_terms(self.fusemap, term_fuses)
                    if terms:
                        d_terms.append(terms)
            if d_terms:
                eqns.append((name, d_terms))
        return eqns

def G16V8(fusemap, pinmap):
    if fusemap[2192] == 1 or fusemap[2193] == 0:
        return G16V8AS(fusemap, pinmap)
    elif fusemap[2192] == 0 or fusemap[2193] != 1:
        return G16V8MS(fusemap, pinmap)
    elif fusemap[2192] == 1 or fusemap[2193] != 1:
        return G16V8MA(fusemap, pinmap)
    else:
        raise TypeError("incorrect device chosen")


class CUPLPrinter:
    def stringize_and(self, items):
        return ' & '.join(items)

    def stringize_or(self, items):
        return '\n  # '.join(items)

    def stringize_list(self, value, names):
        if isinstance(value, ANDList):
            return self.stringize_and([self.stringize_list(x, names) for x in value])
        elif isinstance(value, ORList):
            return self.stringize_or([self.stringize_list(x, names) for x in value])
        else:
            return names[value]

    def print_eqn(self, name, items, names):
        print(name+" =\n    "+self.stringize_list(items, names)+";")

    def print_pin(self, number, signal):
        print("Pin %2d = %s;" % (number, signal))

    def print_comment(self, text):
        print("/* "+text+" */")


def generate_printout(device, printer):
    for pin in range(1, device.pin_count+1):
        name = device.get_pin_name(pin)
        if name:
            if device.is_out_inverted(pin):
                name = inverted(name)
            printer.print_pin(pin, name)
    print()

    names = device.get_input_names()
    for config in device.macrocell_config:
        eqns = device.get_macrocell_eqns(config)
        if eqns:
            for cmt in device.get_macrocell_cmt(config):
                printer.print_comment(cmt)
            for eqn in eqns:
                printer.print_eqn(eqn[0], eqn[1], names)
            print()

def read_pin_map(path):
    try:
        with open(path, "r") as fp:
            lines = fp.readlines()
    except IOError:
        return None
    lines = [line.rstrip() for line in lines]
    for i in range(len(lines)):
        if not lines[i]:
            lines[i] = "PIN%02d" % (i+1)
    return lines

device_type_map = {
    "G16V8": G16V8,
    "GAL16V8": G16V8,
    "G16V8AS": G16V8AS,
    "G16V8MA": G16V8MA,
    "G16V8MS": G16V8MS,
    "PALCE16V8": G16V8,
    "P16L8": P16L8,
    "PAL16L8": P16L8,
}

def main():
    jedec_path = sys.argv[2]
    fusemap = read_jedec(jedec_path)
    pinmap = read_pin_map(os.path.splitext(jedec_path)[0] + '.pin')
    device_name = sys.argv[1].upper()
    try:
        device_type = device_type_map[device_name]
    except KeyError:
        raise ValueError("unknown device "+device_name)
    device = device_type(fusemap, pinmap)
    printer = CUPLPrinter()
    generate_printout(device, printer)

if __name__ == '__main__':
    main()
    