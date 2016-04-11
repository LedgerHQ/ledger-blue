#!/usr/bin/env python
# Copyright (c) 2002-2008 ActiveState Software Inc.
# License: MIT License (http://www.opensource.org/licenses/mit-license.php)

"""
    Preprocess a file.

    Command Line Usage:
        preprocess [<options>...] <infile>

    Options:
        -h, --help      Print this help and exit.
        -V, --version   Print the version info and exit.
        -v, --verbose   Give verbose output for errors.

        -o <outfile>    Write output to the given file instead of to stdout.
        -f, --force     Overwrite given output file. (Otherwise an IOError
                        will be raised if <outfile> already exists.
        -D <define>     Define a variable for preprocessing. <define>
                        can simply be a variable name (in which case it
                        will be true) or it can be of the form
                        <var>=<val>. An attempt will be made to convert
                        <val> to an integer so "-D FOO=0" will create a
                        false value.
        -I <dir>        Add an directory to the include path for
                        #include directives.

        -k, --keep-lines    Emit empty lines for preprocessor statement
                        lines and skipped output lines. This allows line
                        numbers to stay constant.
        -s, --substitute    Substitute defines into emitted lines. By
                        default substitution is NOT done because it
                        currently will substitute into program strings.
        -C, --content-types-path <path>
                        Specify a path to a content.types file to assist
                        with filetype determination. See the
                        `_gDefaultContentTypes` string in this file for
                        details on its format.
        -c, --in-comments 
                        Process preprocessing macros with comments for
                        the first level (not in comments for included 
                        files)
        -e, --in-comments-other-levels 
                        Process preprocessing macros within comments 
                        in included files.
        -r, --ignore <var>
                        The <var> will not be interpreted in
                        conditional blocks, useful to keep dual blocks 
                        such as #if LITTLE_ENDIAN.
        -m, --macro <macroname>
                        The macro defined by <macroname> will be replaced
                        by its definition within the source code
        -R, --only-replace-macro
                        Only replace macro while performing preprocessing.
                        Replaced macros definitions are not printed in output 
                        file as they have been interpreted.
        -d, --discard-prepro-clauses
                        Remove preprocessor clauses after their evaluation.
                        Ifdef/endif/else/elif etc, are interpreted depending 
                        on condition, and not emitted in the output file.
                        
        -p, --dependency <path> 
                        Require the preprocessor to create a GNU Make dependency file (.d)
                        with every included path while preprocessing. To make easier the
                        dependency management at source level.


    Module Usage:
        from preprocess import preprocess
        preprocess(infile, outfile=sys.stdout, defines={}, force=0,
                   keepLines=0, includePath=[], substitute=0,
                   contentType=None)

    The <infile> can be marked up with special preprocessor statement lines
    of the form:
        <comment-prefix> <preprocessor-statement> <comment-suffix>
    where the <comment-prefix/suffix> are the native comment delimiters for
    that file type. 


    Examples
    --------

    HTML (*.htm, *.html) or XML (*.xml, *.kpf, *.xul) files:

        <!-- #if FOO -->
        ...
        <!-- #endif -->
    
    Python (*.py), Perl (*.pl), Tcl (*.tcl), Ruby (*.rb), Bash (*.sh),
    or make ([Mm]akefile*) files:

        # #if defined('FAV_COLOR') and FAV_COLOR == "blue"
        ...
        # #elif FAV_COLOR == "red"
        ...
        # #else
        ...
        # #endif

    C (*.c, *.h), C++ (*.cpp, *.cxx, *.cc, *.h, *.hpp, *.hxx, *.hh),
    Java (*.java), PHP (*.php) or C# (*.cs) files:

        // #define FAV_COLOR 'blue'
        ...
        /* #ifndef FAV_COLOR */
        ...
        // #endif

    Fortran 77 (*.f) or 90/95 (*.f90) files:

        C     #if COEFF == 'var'
              ...
        C     #endif

    And other languages.


    Preprocessor Syntax
    -------------------

    - Valid statements:
        #define <var> [<value>]
        #undef <var>
        #ifdef <var>
        #ifndef <var>
        #if <expr>
        #elif <expr>
        #else
        #endif
        #error <error string>
        #warning <warning string>
        #include "<file>"
        #include <var>
        #inline "<file>" (include always inserting lines whatever the nesting level)
      where <expr> is any valid Python expression.
    - The expression after #if/elif may be a Python statement. It is an
      error to refer to a variable that has not been defined by a -D
      option or by an in-content #define.
    - Special built-in methods for expressions:
        defined(varName)    Return true if given variable is defined.  


    Tips
    ----

    A suggested file naming convention is to let input files to
    preprocess be of the form <basename>.p.<ext> and direct the output
    of preprocess to <basename>.<ext>, e.g.:
        preprocess -o foo.py foo.p.py
    The advantage is that other tools (esp. editors) will still
    recognize the unpreprocessed file as the original language.
"""

__version_info__ = (1, 1, 0)
__version__ = '.'.join(map(str, __version_info__))

import os
import sys
import getopt
import types
import re
import pprint
import string



#---- exceptions

class PreprocessError(Exception):
    def __init__(self, errmsg, file=None, lineno=None, line=None):
        self.errmsg = str(errmsg)
        self.file = file
        self.lineno = lineno
        self.line = line
        Exception.__init__(self, errmsg, file, lineno, line)
    def __str__(self):
        s = ""
        if self.file is not None:
            s += self.file + ":"
        if self.lineno is not None:
            s += str(self.lineno) + ":"
        if self.file is not None or self.lineno is not None:
            s += " "
        s += self.errmsg
        #if self.line is not None:
        #    s += ": " + self.line
        return s



#---- global data

# Comment delimiter info.
#   A mapping of content type to a list of 2-tuples defining the line
#   prefix and suffix for a comment. Each prefix or suffix can either
#   be a string (in which case it is transformed into a pattern allowing
#   whitespace on either side) or a compiled regex.
_commentGroups = {
    "Python":     [ ('#', '') ],
    "Perl":       [ ('#', '') ],
    "PHP":        [ ('/*', '*/'), ('//', ''), ('#', '') ],
    "Ruby":       [ ('#', '') ],
    "Tcl":        [ ('#', '') ],
    "Shell":      [ ('#', '') ],
    # Allowing for CSS and JavaScript comments in XML/HTML.
    "XML":        [ ('<!--', '-->'), ('/*', '*/'), ('//', '') ],
    "HTML":       [ ('<!--', '-->'), ('/*', '*/'), ('//', '') ],
    "Makefile":   [ ('#', '') ],
    "JavaScript": [ ('/*', '*/'), ('//', '') ],
    "CSS":        [ ('/*', '*/') ],
    "C":          [ ('/*', '*/') ],
    "C++":        [ ('/*', '*/'), ('//', '') ],
    "Java":       [ ('/*', '*/'), ('//', '') ],
    "C#":         [ ('/*', '*/'), ('//', '') ],
    "IDL":        [ ('/*', '*/'), ('//', '') ],
    "Text":       [ ('#', '') ],
    "Fortran":    [ (re.compile(r'^[a-zA-Z*$]\s*'), ''), ('!', '') ],
    "TeX":        [ ('%', '') ],
}



#---- internal logging facility

class _Logger:
    DEBUG, INFO, WARN, ERROR, CRITICAL = range(5)
    def __init__(self, name, level=None, streamOrFileName=sys.stderr):
        self._name = name
        if level is None:
            self.level = self.WARN
        else:
            self.level = level
        if type(streamOrFileName) == types.StringType:
            self.stream = open(streamOrFileName, 'w')
            self._opennedStream = 1
        else:
            self.stream = streamOrFileName
            self._opennedStream = 0
    def __del__(self):
        if self._opennedStream:
            self.stream.close()
    def getLevel(self):
        return self.level
    def setLevel(self, level):
        self.level = level
    def _getLevelName(self, level):
        levelNameMap = {
            self.DEBUG: "DEBUG",
            self.INFO: "INFO",
            self.WARN: "WARN",
            self.ERROR: "ERROR",
            self.CRITICAL: "CRITICAL",
        }
        return levelNameMap[level]
    def isEnabled(self, level):
        return level >= self.level
    def isDebugEnabled(self): return self.isEnabled(self.DEBUG)
    def isInfoEnabled(self): return self.isEnabled(self.INFO)
    def isWarnEnabled(self): return self.isEnabled(self.WARN)
    def isErrorEnabled(self): return self.isEnabled(self.ERROR)
    def isFatalEnabled(self): return self.isEnabled(self.FATAL)
    def log(self, level, msg, *args):
        if level < self.level:
            return
        message = "%s: %s: " % (self._name, self._getLevelName(level).lower())
        message = message + (msg % args) + "\n"
        self.stream.write(message)
        self.stream.flush()
    def debug(self, msg, *args):
        self.log(self.DEBUG, msg, *args)
    def info(self, msg, *args):
        self.log(self.INFO, msg, *args)
    def warn(self, msg, *args):
        self.log(self.WARN, msg, *args)
    def error(self, msg, *args):
        self.log(self.ERROR, msg, *args)
    def fatal(self, msg, *args):
        self.log(self.CRITICAL, msg, *args)

log = _Logger("preprocess", _Logger.WARN)


definedexprtopy = re.compile('defined\s*\(\s*([^\s\)\']+)\s*\)')
definedexprtopy_alt = re.compile('defined\s*([^\s\'\(\)]+)\s*')
suffixedvalue = re.compile('(^|\s+)(0x[0-9A-Fa-f]+|[0-9]+)[UL]*')
def _cpp_to_python(expr):
    repl = string.replace(expr, '!=', ' notequal ')
    repl = string.replace(repl, '!', ' not ')
    repl = string.replace(repl, ' notequal ', '!=')
    repl = string.replace(repl, '||', ' or ')
    repl = string.replace(repl, '&&', ' and ')
    repl = definedexprtopy_alt.sub(r"defined('\1')",repl)
    repl = definedexprtopy.sub(r"defined('\1')",repl)
    repl = suffixedvalue.sub(r'\1\2',repl)
    return repl

definedexprfrompy = re.compile('defined\(\s*\'\s*([^\)\']+)\s*\'\s*\)')
def _python_to_cpp(expr):
    repl = string.replace(expr, ' not ', ' !')
    repl = string.replace(repl, ' or ', '||')
    repl = string.replace(repl, ' and ', '&&')
    return definedexprfrompy.sub(r"defined(\1)",repl)    
    
#---- internal support stuff

def _evaluate(expr, defines, onlyMacroReplace, replaceDefinesBeforeEval=True):
    """Evaluate the given expression string with the given context.

    WARNING: This runs eval() on a user string. This is unsafe.
    """

    # perform replacement while they can be done
    if replaceDefinesBeforeEval:
      while True:
        log.debug(expr)
        expr_old = expr
        expr = _replace_defines(expr, defines)
        # stop replacing when no replace happened
        if expr == expr_old:
          break

    #interpolated = _interpolate(s, defines)
    expr = _cpp_to_python(expr)
    while True:
      try:
          rv = eval(expr, {'defined':lambda v: v in defines}, defines)
      except Exception, ex:
          if not onlyMacroReplace:
            msg = str(ex)
            if msg.startswith("name '") and msg.endswith("' is not defined"):
                # A common error (at least this is presumed:) is to have
                #   defined(FOO)   instead of   defined('FOO')
                # We should give a little as to what might be wrong.
                # msg == "name 'FOO' is not defined"  -->  varName == "FOO"
                varName = msg[len("name '"):-len("' is not defined")]
                if expr.find("defined(%s)" % varName) != -1:
                    # "defined(FOO)" in expr instead of "defined('FOO')"
                    msg += " (perhaps you want \"defined('%s')\" instead of "\
                          "\"defined(%s)\")" % (varName, varName)
            elif msg.startswith("global name '") and msg.endswith("' is not defined"):
                # A common error (at least this is presumed:) is to have
                #   defined(FOO)   instead of   defined('FOO')
                # We should give a little as to what might be wrong.
                # msg == "name 'FOO' is not defined"  -->  varName == "FOO"
                varName = msg[len("global name '"):-len("' is not defined")]
                if expr.find("defined(%s)" % varName) != -1:
                    # "defined(FOO)" in expr instead of "defined('FOO')"
                    msg += " (perhaps you want \"defined('%s')\" instead of "\
                          "\"defined(%s)\")" % (varName, varName)
                # Temporarily do as the CPP parser does, when an unknown token is found, the define is just ignored, and condition goes False
                return False
            elif msg.startswith("invalid syntax"):
                msg = "invalid syntax: '%s'" % expr
            raise PreprocessError(msg, defines['__FILE__'], defines['__LINE__'])
          else:
            return False
      log.debug("evaluate %r -> %s (defines=%r)", expr, rv, defines)
      break
    return rv
  
class Macro:
  def __init__(self, name, args, code):
    log.debug("new macro: " +str(name) +" args: " + str(args) + " code: " +str(code))
    self.name = name
    self.args = args
    self.code = code
    self.headre = re.compile('(?P<before>.*?[^\w])?(?P<start>'+name+'\s*\()')
    
class Define:
  def __init__(self, var, val):
    log.debug("new define: " +str(var) +" val: " + str(val))
    self.name = var
    self.args = []
    self.code = val
    self.headre = re.compile('(?P<before>.*?[^\w])?(?P<start>'+var+'\s*)')

#def _split_args(line, offset=0, args=[], token="", parenthesis=1):
def _split_args(line):
  offset=0
  args=[]
  token=""
  parenthesis=1
  # consider we start right after the open parenthesis of the macro
  # split arguments (take into account parenthesis and comma)
  while offset < len(line):
    char = line[offset]
    if (char == '('):
      parenthesis+=1
      token += char
    elif (char == ')'):
      parenthesis-=1
      # end of macro
      if (parenthesis==0):
        args.append(token)
        token=""
        break
      else:
        token += char
    # higher order of parameter are considered argument of nested macros
    elif (char == ',' and parenthesis == 1):
      args.append(token)
      token=""
    else:
      token += char
    offset+=1
  # return the end index (of the last parenthesis or EOL)
  #return offset, args, token, parenthesis
  return args, offset

class NotEnoughArgsException:
  pass

def _replace_macros(line, macros):
  matched = True # ensure entering the first time
  while matched:
    # when current line matches no macro then finish processing current line
    matched = False
    for macro in macros:
      # fast skip if macro name is not present
      if line.find(macro.name) == -1:
        continue
      
      # match macro name and starting parenthesis
      match = macro.headre.match(line)
      if match:
        matched=True
        
        if len(macro.args) > 0:
          # grab arguments and the end offset to know what to replace
          args, offset = _split_args(line[match.end('start'):])
          
          # replace line[match.start('start'):offset] (the matching macro start:stop)
          prepos = match.start('start')
          #if (prepos == 0):
          #  prepos -= 1
          postpos = match.end('start')+offset+1
          
          # replace arguments with argument instances
          code = macro.code
          try:
            for i in range(0, len(macro.args)):
              code = code.replace(macro.args[i], args[i])
          except:
            raise NotEnoughArgsException
          
          # replace in current line
          line = line[:prepos] + code + line[postpos:]
        else:
          # directly replace macro
          if macro.code:
            line = line.replace(macro.name, macro.code)
          else:
            line = line.replace(macro.name, "")
        
        # reparse the line from the beginning
  return line



commentsremulti = re.compile('/\*(.*\*)?/')
commentsresingle = re.compile('//.*$')
def _replace_comments(line, contentType):
  line = commentsremulti.sub("",line)
  line = commentsresingle.sub("",line)
  return line
  

#---- module API
'''
# C define syntax (inverted to split on non-define tokens)
defineNameRE = re.compile(r'[^\w_]+')
def _replace_defines(oline, defines, dont_replace_empty_define=True):
  line = oline
  
  # TODO add an option to replace define only if not within a defined(x) clause (to replace only when doing aruthmetic in macro)
  
  parts = re.split(defineNameRENotInDefined,line)
  if parts and len(parts) > 0:
    for part in parts:
      if part in defines:
        if dont_replace_empty_define and (defines[part] == "\'None\'" or not defines[part]):
          continue
        line =re.sub(part,str(defines[part]),line)
        #line  = line.replace(part,str(defines[part]))
  log.debug("before define replace:" + oline)
  log.debug("after define replace: " + line)
  return line
'''

defineTokenRE = re.compile(r'\w+')
definedRE = re.compile(r'(?:[^\w])defined\s*\(\s*$')
defineRE = re.compile(r'^\s*#\s*define\s+$')

def _replace_defines(oline, defines, dont_replace_empty_define=True):
  line = oline
  
  #assert defineTokenRE.match(oline)
  
  pos=0
  while True:
    m = defineTokenRE.search(line, pos)
    #log.debug(line)
    #if pos:
    #  log.debug(" " * pos + "^\n")
    #else:
    #  log.debug("^\n")

    # no more matches to replace a define
    if m:
      define = m.group()
      if define in defines:
        defvalue = str(defines[define])
        
        if dont_replace_empty_define and (defvalue == "None" or defvalue == "\'None\'" or not defvalue):
          #log.debug("skip empty " + define)
          pos = m.end()
        else:
          # check if preceded with defined( to avoid replacing for define checks only
          if not definedRE.search(line, 0, m.start()) and not defineRE.search(line, 0, m.start()):
            #log.debug("replace " + define + " by " + defvalue)
            # replace with the define's value 
            line = line[:m.start()] + defvalue + line[m.end():]
            # avoid rematching the same token
            pos = m.start() + len(defvalue)
          else:
            #log.debug("skip " + define)
            pos = m.end()
      
      # always advance to avoid infinite loop
      pos+=1
    else:
      #log.debug("no replaceable define found")
      break

  log.debug("before define replace:" + oline)
  log.debug("after define replace: " + line)
  return line

def preprocess(infile, outfile=sys.stdout, defines={},
               force=0, keepLines=0, includePath=[], substitute=0, 
               contentType=None, contentTypesRegistry=None,
               firstLevel=True,preprocessComments=0,preprocessCommentsOtherLevels=0,
               ignoredDefines=[], macros=[], replacedMacros=[], onlyMacroReplace=False, printPPClauses=True, dependencies=[]):
  
    """Preprocess the given file.

    "infile" is the input path.
    "outfile" is the output path or stream (default is sys.stdout).
    "defines" is a dictionary of defined variables that will be
        understood in preprocessor statements. Keys must be strings and,
        currently, only the truth value of any key's value matters.
    "force" will overwrite the given outfile if it already exists. Otherwise
        an IOError will be raise if the outfile already exists.
    "keepLines" will cause blank lines to be emitted for preprocessor lines
        and content lines that would otherwise be skipped.
    "includePath" is a list of directories to search for given #include or #inline
        directives. The directory of the file being processed is presumed.
    "substitute", if true, will allow substitution of defines into emitted
        lines. (NOTE: This substitution will happen within program strings
        as well. This may not be what you expect.)
    "contentType" can be used to specify the content type of the input
        file. It not given, it will be guessed.
    "contentTypesRegistry" is an instance of ContentTypesRegistry. If not specified
        a default registry will be created.
    "__preprocessedFiles" (for internal use only) is used to ensure files
        are not recusively preprocessed.

    Returns the modified dictionary of defines or raises PreprocessError if
    there was some problem.
    """ 
    log.info("preprocess(infile=%r, outfile=%r, defines=%r, force=%r, "\
             "keepLines=%r, includePath=%r, contentType=%r, )", infile, outfile, defines, force,
             keepLines, includePath, contentType)

    # Determine the content type and comment info for the input file.
    if contentType is None:
        registry = contentTypesRegistry or getDefaultContentTypesRegistry()
        contentType = registry.getContentType(infile)
        log.info("content type is '%s'", contentType)
        if contentType is None:
            contentType = "Text"
            log.warn("defaulting content type for '%s' to '%s'",
                     infile, contentType)
    try:
        cgs = _commentGroups[contentType]
    except KeyError:
        raise PreprocessError("don't know comment delimiters for content "\
                              "type '%s' (file '%s')"\
                              % (contentType, infile))

    # Generate statement parsing regexes. Basic format:
    #       <comment-prefix> <preprocessor-stmt> <comment-suffix>
    #  Examples:
    #       <!-- #if foo -->
    #       ...
    #       <!-- #endif -->
    #
    #       # #if BAR
    #       ...
    #       # #else
    #       ...
    #       # #endif
    stmts = ['#\s*(?P<op>if|elif|ifdef|ifndef)(?:\s+(?P<expr>.+?))',
             '#\s*(?P<op>else|endif)(\s+.*)?', # accept junk after
             '#\s*(?P<op>error)\s+(?P<error>.+?)',
             '#\s*(?P<op>warning)\s+(?P<warning>.+?)',
             '#\s*(?P<op>define)\s+(?P<macro>[\w_\-\+]+?)\s*\(\s*(?P<args>[a-zA-Z_][\w_\-\+\s,]+?)\)\s*(?P<code>.+?)',
             # NOT WORKING '#\s*(?P<op>define)\s+(?P<macro>[\w_\-\+]+)\s*\(\s*(?P<args>([a-zA-Z_][\w_\-\+]+\s*,?\s*)+)\)\s*(?P<code>.+)',
             '#\s*(?P<op>define)\s+(?P<var>[^\s\(]+?)(?:\s+(?P<val>.+?))?',
             '#\s*(?P<op>undef)\s+(?P<var>[^\s]+)',
             '#\s*(?P<op>include)\s+"(?P<fname>.+)"',
             '#\s*(?P<op>include)\s+<(?P<fname>.+)>',
             '#\s*(?P<op>include)\s+(?P<var>[^\s]+)',
             '#\s*(?P<op>inline)\s+"(?P<fname>.+)"',
             '#\s*(?P<op>inline)\s+<(?P<fname>.+)>',
            ]
    patterns = []
    for stmt in stmts:
        # The comment group prefix and suffix can either be just a
        # string or a compiled regex.
        if preprocessComments:
            for cprefix, csuffix in cgs:
                if hasattr(cprefix, "pattern"):
                    pattern = cprefix.pattern
                else:
                    pattern = r"^\s*%s\s*" % re.escape(cprefix)
                pattern += stmt
                if hasattr(csuffix, "pattern"):
                    pattern += csuffix.pattern
                else:
                    pattern += r"\s*%s\s*$" % re.escape(csuffix)
        else:
            # support both kind of comment declaration after the prepro command
            pattern = r"^\s*%s\s*(?:/[\*/].*)?$" % stmt
        patterns.append(pattern)
        log.debug("pattern: '%s'", pattern)
        
    stmtRes = [re.compile(p) for p in patterns]

    # Process the input file.
    # (Would be helpful if I knew anything about lexing and parsing
    # simple grammars.)
    fin = open(infile, 'r')
    lines = fin.readlines()
    fin.close()
    if type(outfile) in types.StringTypes:
        if force and os.path.exists(outfile):
            os.chmod(outfile, 0777)
            os.remove(outfile)
        fout = open(outfile, 'w')
    else:
        fout = outfile

    defines['__FILE__'] = infile
    SKIP, EMIT = range(2) # states
    states = [(EMIT,   # a state is (<emit-or-skip-lines-in-this-section>,
               0,      #             <have-emitted-in-this-if-block>,
               0,      #             <have-seen-'else'-in-this-if-block>)
               '')]      # name of block
    lineNum = 0
    multiline=""
    modeMultiline=False
    for line in lines:
        # ensure no \r to avoid regexp matching problem for final $
        line = line.replace('\r','')
        
        lineNum += 1
        log.debug("line %d: %r", lineNum, line)
        defines['__LINE__'] = lineNum
        
        # macro replacements
        line = _replace_macros(line, macros)
        log.debug("after replace macro %r", line)
          
        # accumulate lines for multiline macro definition
        matchline = line
        if modeMultiline:
          lineStrip = matchline[:-1].strip()
          multiline += lineStrip[:-1]
          # end the multiline joining
          if not lineStrip.endswith("\\"):
            matchline = multiline
            modeMultiline=False

        # avoid multi interpretation of a to be joined line (multiline)
        match = None
        
        # reevaluating if current line exits a multiline def          
        if not modeMultiline:
          # check if line has finished with a \ to join the next line before evaluating (TODO skip comments !!)
          lineStrip = matchline[:-1].strip()
          if lineStrip.endswith("\\"):
            multiline = lineStrip[:-1]
            modeMultiline=True
          
          if not modeMultiline:
              # Is this line a preprocessor stmt line?
              #XXX Could probably speed this up by optimizing common case of
              #    line NOT being a preprocessor stmt line.
              for stmtRe in stmtRes:
                  match = stmtRe.match(matchline)
                  if match:
                      break
              else:
                  match = None

        if match: #{
            printOpLine = True
            # ensure to avoid printing the endif of a section
            op = match.group("op")
            log.debug("%r stmt (states: %r)", op, states)
            if op == "define":
                if not (states and states[-1][0] == SKIP):
                    definekind=0
                    try:
                      if not (len(match.group("macro")) > 0 and len(match.group("args")) > 0 and len(match.group("code")) > 0 ):
                        raise Exception
                      definekind=2
                    except:
                      try:
                        match.group("var", "val")
                        definekind=1
                      except:
                        pass
                    
                    # not skipped
                    if definekind == 1: 
                      log.debug("var+val")
                      var, val = match.group("var", "val")
                      if not var in ignoredDefines:
                          if val is None:
                              val = None
                          else:
                              try:
                                  val = eval(val, {}, {})
                              except:
                                  pass
                          #log.debug(str(var) + " " + str(val))
                          #print str(var) + " " + str(val)
                          defines[var] = val
                          
                          # append the define to the macro list to be replaced if it matches the requested patterns
                          for m in replacedMacros:
                            if re.match(m, var):
                              macros.append(Define(var,val))
                              printOpLine = False
                              log.debug("new define to replace: " + str(var) + " " + str(val))
                              break;
                    elif definekind == 2:                      
                      log.debug("macro")
                      macro = match.group("macro")
                      # a macro is a define at first (avoid problem when checking if a macro is defined or not)
                      defines[macro] = None
                      args = match.group("args").replace(" ", "").split(",")
                      for m in replacedMacros:
                        if re.match(m, macro):
                          macros.append(Macro(macro,args,match.group("code")))
                          # don't print replaced macro
                          printOpLine = False
                          log.debug("new macro to replace: " + str(macro))
                          break;
                    else:
                      log.debug("unknown define kind")
                      raise Exception
                    printOpLine = printPPClauses
                else:
                    log.debug("define skipped")
                    printOpLine = False
            elif op == "undef":
                if not (states and states[-1][0] == SKIP):
                    var = match.group("var")
                    try:
                        del defines[var]
                    except KeyError:
                        pass
                    printOpLine = printPPClauses
                else:
                    printOpLine = False
            elif op == "include":
                if not (states and states[-1][0] == SKIP):
                    skipinclude=0
                    if "var" in match.groupdict():
                        # This is the second include form: #include VAR
                        var = match.group("var")
                        f = defines[var]
                    else:
                        # This is the first include form: #include "path"
                        f = match.group("fname")
                        
                    for d in [os.path.dirname(infile)] + includePath:
                        fname = os.path.normpath(os.path.join(d, f))
                        if os.path.exists(fname):
                            break
                    else:
                        skipinclude=1
                        print "warning: could not find included file \"%s\"" % (f)
                        #raise PreprocessError("could not find #include'd file "\
                        #                      "\"%s\" on include path: %r"\
                        #                      % (f, includePath))
                    if skipinclude!=1:
                        if dependencies:
                          #print "Depencency found: " + fname
                          dependencies.append(fname)
                        preprocess(fname, fout, defines, force,
                                            keepLines, includePath, substitute,
                                            contentTypesRegistry=contentTypesRegistry,
                                            firstLevel=False,
                                            preprocessComments=preprocessCommentsOtherLevels,
                                            preprocessCommentsOtherLevels=preprocessCommentsOtherLevels,
                                            macros=macros, 
                                            replacedMacros=replacedMacros,
                                            onlyMacroReplace=onlyMacroReplace,
                                            printPPClauses=printPPClauses,
                                            dependencies=dependencies)
                        # set back the file name of current context in the defines 
                        # (else wrong display in errors)
                        defines['__FILE__'] = infile
                    printOpLine = printPPClauses
                else:
                    printOpLine = False
            elif op == "inline":
                if not (states and states[-1][0] == SKIP):
                    skipinclude=0
                    if "fname" in match.groupdict():
                        # This is the first include form: #inline "path"
                        f = match.group("fname")
                        
                    for d in [os.path.dirname(infile)] + includePath:
                        fname = os.path.normpath(os.path.join(d, f))
                        if os.path.exists(fname):
                            break
                    else:
                        raise PreprocessError("could not find #inline'd file "\
                                              "\"%s\" on include path: %r"\
                                              % (f, includePath))
                    if dependencies:
                      #print "Depencency found: " + fname
                      dependencies.append(fname)
                    preprocess(fname, fout, defines, force,
                                        keepLines, includePath, substitute,
                                        contentTypesRegistry=contentTypesRegistry,
                                        firstLevel=True, # always consider inline files as first level to ensure emitting lines
                                        preprocessComments=preprocessCommentsOtherLevels,
                                        preprocessCommentsOtherLevels=preprocessCommentsOtherLevels,
                                        macros=macros, 
                                        replacedMacros=replacedMacros,
                                        onlyMacroReplace=onlyMacroReplace,
                                        printPPClauses=printPPClauses,
                                        dependencies=dependencies)
                    # set back the file name of current context in the defines 
                    # (else wrong display in errors)
                    defines['__FILE__'] = infile
                else:
                    printOpLine = False
            elif op in ("if", "ifdef", "ifndef"):
                replaceDefines=False
                if op == "if":
                    expr = match.group("expr")
                    replaceDefines = True
                elif op == "ifdef":
                    expr = "defined('%s')" % match.group("expr")
                elif op == "ifndef":
                    expr = "not defined('%s')" % match.group("expr")
                
                ignored = False
                for ignoredDef in ignoredDefines:
                    if expr.find(ignoredDef) != -1 or re.match(ignoredDef,expr):
                        ignored=True
                        break
                
                if ignored and states and states[-1][0] != SKIP:
                    #special emit state to notify every part is to be printed
                    states.append((EMIT, 2, 0, expr))
                else:
                    try:
                        if states and states[-1][0] == SKIP:
                            # Were are nested in a SKIP-portion of an if-block.
                            states.append((SKIP, 0, 0, expr))
                            printOpLine = False
                        elif _evaluate(expr, defines, onlyMacroReplace, replaceDefines):
                            states.append((EMIT, 1, 0, expr))
                            printOpLine = printPPClauses
                        else:
                            states.append((SKIP, 0, 0, expr))
                            printOpLine = False
                    except KeyError:
                        raise PreprocessError("use of undefined variable in "\
                                              "#%s stmt" % op, defines['__FILE__'],
                                              defines['__LINE__'], line)
            elif op == "elif":
                expr = match.group("expr")
                try:
                    if states[-1][2]: # already had #else in this if-block
                        raise PreprocessError("illegal #elif after #else in "\
                            "same #if block", defines['__FILE__'],
                            defines['__LINE__'], line)
                    elif states[-1][1] == 2:
                        #emit all blocks as the first condition contains an ignored define
                        printOpLine = True
                        pass
                    elif states[-1][1]: # if have emitted in this if-block
                        states[-1] = (SKIP, 1, 0, expr)
                        printOpLine = False
                    elif states[:-1] and states[-2][0] == SKIP:
                        # Were are nested in a SKIP-portion of an if-block.
                        states[-1] = (SKIP, 0, 0, expr)
                        printOpLine = False
                    elif _evaluate(expr, defines, onlyMacroReplace):
                        states[-1] = (EMIT, 1, 0, expr)
                        # replace elif by if to ensure produced source is compilable
                        p = re.compile('elif')
                        line = p.sub(r'if',line)
                        printOpLine = printPPClauses
                    else:
                        states[-1] = (SKIP, 0, 0, expr)
                        printOpLine = False
                except IndexError:
                    raise PreprocessError("#elif stmt without leading #if "\
                                          "stmt", defines['__FILE__'],
                                            defines['__LINE__'], line)
            elif op == "else":
                try:
                    expr = states[-1][3]
                    if states[-1][2]: # already had #else in this if-block
                        raise PreprocessError("illegal #else after #else in "\
                            "same #if block", defines['__FILE__'],
                            defines['__LINE__'], line)
                    elif states[-1][1] == 2:
                        #emit all blocks as the first condition contains an ignored define
                        printOpLine = True
                        pass
                    elif states[-1][1]: # if have emitted in this if-block
                        states[-1] = (SKIP, 1, 1, expr)
                        printOpLine = False
                    elif states[:-1] and states[-2][0] == SKIP:
                        # Were are nested in a SKIP-portion of an if-block.
                        states[-1] = (SKIP, 0, 1, expr)
                        printOpLine = False
                    else:
                        # replace else by if to ensure produced source is compilable
                        p = re.compile('else')
                        line = p.sub("if ! ( " + _python_to_cpp(expr) + " )",line)
                        states[-1] = (EMIT, 1, 1, expr)
                        printOpLine = printPPClauses
                except IndexError:
                    raise PreprocessError("#else stmt without leading #if "\
                                          "stmt", defines['__FILE__'],
                                          defines['__LINE__'], line)
            elif op == "endif":
                try:
                    printOpLine = printPPClauses
                    
                    # if no block emitted for the if-elif*-else, then skip the endif
                    if not states[-1][1]:
                        printOpLine = False
                        
                    # emit all pp clauses as an ignored define has been encountered
                    if states[-1][1] == 2:
                        printOpLine = True
                        
                    states.pop()
                except IndexError:
                    raise PreprocessError("#endif stmt without leading #if"\
                                          "stmt", defines['__FILE__'],
                                          defines['__LINE__'], line)
            elif op == "error":
                # if line is within an ignored define block
                if states[-1][1] == 2:
                    printOpLine = True
                # if line is skipped 
                elif not (states and states[-1][0] == SKIP):
                    error = match.group("error")
                    if not onlyMacroReplace:
                      raise PreprocessError("#error: "+error, defines['__FILE__'],
                                            defines['__LINE__'], line)
                else:
                    printOpLine = False
          
            log.debug("states: %r", states)
            # only echo directives if emitting and for the first level (included files are not inlined)
            if firstLevel:
                if printOpLine or onlyMacroReplace:
                    log.debug("emit prepro line (%s)" % line)
                    fout.write(line)
                else:
                    # keep line number constant
                    if keepLines:
                        fout.write("\n")
                    log.debug("skip prepro line (%s)" % line)
            #}
        else:
            try:
                # only print lines of the top level processed file (we're not cpp :p)
                if firstLevel:
                    if states[-1][0] == EMIT or onlyMacroReplace:
                        log.debug("emit line (%s)" % states[-1][1])
                        # Substitute all defines into line.
                        # XXX Should avoid recursive substitutions. But that
                        #     would be a pain right now.
                        sline = line
                        if substitute:
                            sline = _replace_defines(sline, defines)
                            """
                            for name in reversed(sorted(defines, key=len)):
                                value = defines[name]
                                #print str(len(sline))+ ":" + sline + " repl:" + name
                                pattern=r'(^|[^\w_])'+str(name)+r'([^\w_]|$)'
                                repl=r'\1'+str(value)+r'\2'
                                print pattern
                                print repl
                                sline = re.sub(pattern,repl,sline)
                                #sline = sline.replace(name, str(value))
                            """
                        fout.write(sline)
                    elif keepLines:
                        log.debug("keep blank line (%s)" % states[-1][1])
                        fout.write("\n")
                    else:
                        log.debug("skip line (%s)" % states[-1][1])
                else:
                    log.debug("skip line not first level (%s)" % firstLevel)
            except IndexError:
                raise PreprocessError("superfluous #endif before this line",
                                      defines['__FILE__'],
                                      defines['__LINE__'])
    if len(states) > 1:
        raise PreprocessError("unterminated block \"%s\"" % (states[-1][3]), defines['__FILE__'],
                              defines['__LINE__'])
    elif len(states) < 1:
        raise PreprocessError("superfluous #endif on or before this line",
                              defines['__FILE__'], defines['__LINE__'])

    if fout != outfile:
        fout.close()


#---- content-type handling

_gDefaultContentTypes = """
    # Default file types understood by "preprocess.py".
    #
    # Format is an extension of 'mime.types' file syntax.
    #   - '#' indicates a comment to the end of the line.
    #   - a line is:
    #       <filetype> [<pattern>...]
    #     where,
    #       <filetype>'s are equivalent in spirit to the names used in the Windows
    #           registry in HKCR, but some of those names suck or are inconsistent;
    #           and
    #       <pattern> is a suffix (pattern starts with a '.'), a regular expression
    #           (pattern is enclosed in '/' characters), a full filename (anything
    #           else).
    #
    # Notes on case-sensitivity:
    #
    # A suffix pattern is case-insensitive on Windows and case-sensitive
    # elsewhere.  A filename pattern is case-sensitive everywhere. A regex
    # pattern's case-sensitivity is defined by the regex. This means it is by
    # default case-sensitive, but this can be changed using Python's inline
    # regex option syntax. E.g.:
    #         Makefile            /^(?i)makefile.*$/   # case-INsensitive regex

    Python              .py
    Python              .pyw
    Perl                .pl
    Ruby                .rb
    Tcl                 .tcl
    XML                 .xml
    XML                 .kpf
    XML                 .xul
    XML                 .rdf
    XML                 .xslt
    XML                 .xsl
    XML                 .wxs
    XML                 .wxi
    HTML                .htm
    HTML                .html
    XML                 .xhtml
    Makefile            /^[Mm]akefile.*$/
    PHP                 .php
    JavaScript          .js
    CSS                 .css
    C++                 .c       # C++ because then we can use //-style comments
    C++                 .cpp
    C++                 .cxx
    C++                 .cc
    C++                 .h
    C++                 .hpp
    C++                 .hxx
    C++                 .hh
    IDL                 .idl
    Text                .txt
    Fortran             .f
    Fortran             .f90
    Shell               .sh
    Shell               .csh
    Shell               .ksh
    Shell               .zsh
    Java                .java
    C#                  .cs
    TeX                 .tex

    # Some Komodo-specific file extensions
    Python              .ksf  # Fonts & Colors scheme files
    Text                .kkf  # Keybinding schemes files
"""

class ContentTypesRegistry:
    """A class that handles determining the filetype of a given path.

    Usage:
        >>> registry = ContentTypesRegistry()
        >>> registry.getContentType("foo.py")
        "Python"
    """

    def __init__(self, contentTypesPaths=None):
        self.contentTypesPaths = contentTypesPaths
        self._load()

    def _load(self):
        from os.path import dirname, join, exists

        self.suffixMap = {}
        self.regexMap = {}
        self.filenameMap = {}

        self._loadContentType(_gDefaultContentTypes)
        localContentTypesPath = join(dirname(__file__), "content.types")
        if exists(localContentTypesPath):
            log.debug("load content types file: `%r'" % localContentTypesPath)
            self._loadContentType(open(localContentTypesPath, 'r').read())
        for path in (self.contentTypesPaths or []):
            log.debug("load content types file: `%r'" % path)
            self._loadContentType(open(path, 'r').read())

    def _loadContentType(self, content, path=None):
        """Return the registry for the given content.types file.
       
        The registry is three mappings:
            <suffix> -> <content type>
            <regex> -> <content type>
            <filename> -> <content type>
        """
        for line in content.splitlines(0):
            words = line.strip().split()
            for i in range(len(words)):
                if words[i][0] == '#':
                    del words[i:]
                    break
            if not words: continue
            contentType, patterns = words[0], words[1:]
            if not patterns:
                if line[-1] == '\n': line = line[:-1]
                raise PreprocessError("bogus content.types line, there must "\
                                      "be one or more patterns: '%s'" % line)
            for pattern in patterns:
                if pattern.startswith('.'):
                    if sys.platform.startswith("win"):
                        # Suffix patterns are case-insensitive on Windows.
                        pattern = pattern.lower()
                    self.suffixMap[pattern] = contentType
                elif pattern.startswith('/') and pattern.endswith('/'):
                    self.regexMap[re.compile(pattern[1:-1])] = contentType
                else:
                    self.filenameMap[pattern] = contentType

    def getContentType(self, path):
        """Return a content type for the given path.

        @param path {str} The path of file for which to guess the
            content type.
        @returns {str|None} Returns None if could not determine the
            content type.
        """
        basename = os.path.basename(path)
        contentType = None
        # Try to determine from the path.
        if not contentType and self.filenameMap.has_key(basename):
            contentType = self.filenameMap[basename]
            log.debug("Content type of '%s' is '%s' (determined from full "\
                      "path).", path, contentType)
        # Try to determine from the suffix.
        if not contentType and '.' in basename:
            suffix = "." + basename.split(".")[-1]
            if sys.platform.startswith("win"):
                # Suffix patterns are case-insensitive on Windows.
                suffix = suffix.lower()
            if self.suffixMap.has_key(suffix):
                contentType = self.suffixMap[suffix]
                log.debug("Content type of '%s' is '%s' (determined from "\
                          "suffix '%s').", path, contentType, suffix)
        # Try to determine from the registered set of regex patterns.
        if not contentType:
            for regex, ctype in self.regexMap.items():
                if regex.search(basename):
                    contentType = ctype
                    log.debug("Content type of '%s' is '%s' (matches regex '%s')",
                              path, contentType, regex.pattern)
                    break
        # Try to determine from the file contents.
        content = open(path, 'rb').read()
        if content.startswith("<?xml"):  # cheap XML sniffing
            contentType = "XML"
        return contentType

_gDefaultContentTypesRegistry = None
def getDefaultContentTypesRegistry():
    global _gDefaultContentTypesRegistry
    if _gDefaultContentTypesRegistry is None:
        _gDefaultContentTypesRegistry = ContentTypesRegistry()
    return _gDefaultContentTypesRegistry


#---- internal support stuff
#TODO: move other internal stuff down to this section

try:
    reversed
except NameError:
    # 'reversed' added in Python 2.4 (http://www.python.org/doc/2.4/whatsnew/node7.html)
    def reversed(seq):
        rseq = list(seq)
        rseq.reverse()
        for item in rseq:
            yield item
try:
    sorted
except NameError:
    # 'sorted' added in Python 2.4. Note that I'm only implementing enough
    # of sorted as is used in this module.
    def sorted(seq, key=None):
        identity = lambda x: x
        key_func = (key or identity)
        sseq = list(seq)
        sseq.sort(lambda self, other: cmp(key_func(self), key_func(other)))
        for item in sseq:
            yield item


#---- mainline

def main(argv):
    try:
        optlist, args = getopt.getopt(argv[1:], 'hVvo:D:fkI:sC:cer:m:Rdp:',
            ['help', 'version', 'verbose', 'force', 'keep-lines',
             'substitute', 'content-types-path=', '--in-comments', 
             '--in-comments-other-levels','--ignore=','--macro=','--only-replace-macro','--discard-prepro-clauses','--dependency='])
    except getopt.GetoptError, msg:
        sys.stderr.write("preprocess: error: %s. Your invocation was: %s\n"\
                         % (msg, argv))
        sys.stderr.write("See 'preprocess --help'.\n")
        return 1
    outfile = sys.stdout
    defines = {}
    force = 0
    keepLines = 0
    substitute = 0
    preprocessComments = 0
    preprocessCommentsOtherLevels = 0
    includePath = []
    contentTypesPaths = []
    ignorePattern = []
    replacedMacros = []
    macroReplace = False
    printPPClauses=True
    dependencies=[]
    depencenciesOutput=None
    for opt, optarg in optlist:
        if opt in ('-h', '--help'):
            sys.stdout.write(__doc__)
            return 0
        elif opt in ('-V', '--version'):
            sys.stdout.write("preprocess %s\n" % __version__)
            return 0
        elif opt in ('-v', '--verbose'):
            log.setLevel(log.DEBUG)
        elif opt == '-o':
            outfile = optarg
        if opt in ('-f', '--force'):
            force = 1
        elif opt == '-D':
            if optarg.find('=') != -1:
                var, val = optarg.split('=', 1)
                try:
                    val = int(val)
                except ValueError:
                    pass
            else:
                var, val = optarg, None
            defines[var] = val
        elif opt in ('-k', '--keep-lines'):
            keepLines = 1
        elif opt == '-I':
            includePath.append(optarg)
        elif opt in ('-s', '--substitute'):
            substitute = 1
        elif opt in ('-C', '--content-types-path'):
            contentTypesPaths.append(optarg)
        elif opt in ('-c', '--in-comments'):
            preprocessComments=1
        elif opt in ('-e', '--in-comments-other-levels'):
            preprocessCommentsOtherLevels=1
        elif opt in ('-r', '--ignore'):
            ignorePattern.append(optarg)
        elif opt in ('-m', '--macro'):
            replacedMacros.append(optarg)
        elif opt in ('-R','--only-replace-macro'):
            print "replace only macro: disabled buggy option"
            sys.exit(-1)
            macroReplace = True
        elif opt in ('-d','--discard-prepro-clauses'):
            printPPClauses = False
        elif opt in ('-p','--dependency'):
            depencenciesOutput = optarg

    if len(args) != 1:
        sys.stderr.write("preprocess: error: incorrect number of "\
                         "arguments: argv=%r\n" % argv)
        return 1
    else:
        infile = args[0]

    try:
        contentTypesRegistry = ContentTypesRegistry(contentTypesPaths)

        #print "Depencency found: " + infile
        dependencies.append(infile)
        preprocess(infile, outfile, defines, force, keepLines, includePath,
                   substitute, contentTypesRegistry=contentTypesRegistry, 
                   preprocessComments=preprocessComments, preprocessCommentsOtherLevels=preprocessCommentsOtherLevels, ignoredDefines=ignorePattern,
                   replacedMacros=replacedMacros, onlyMacroReplace=macroReplace,printPPClauses=printPPClauses,dependencies=dependencies)
        
        # create dependencies file if requested
        if depencenciesOutput:
          depf = open(depencenciesOutput,"w+")
          # infile has not been included in the dependencies yet, do it
          depf.write(outfile + ": ")
          for dep in dependencies:
            depf.write(dep + " ")
          depf.write("\n")
          depf.close()
          
    except PreprocessError, ex:
        if log.isDebugEnabled():
            import traceback
            traceback.print_exc(file=sys.stderr)
        else:
            sys.stderr.write("preprocess: error: %s\n" % str(ex))
        return 1

if __name__ == "__main__":
    __file__ = sys.argv[0]
    sys.exit( main(sys.argv) )

