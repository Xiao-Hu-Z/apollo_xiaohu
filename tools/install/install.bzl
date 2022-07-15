# -*- python -*-
# Adapted from RobotLocomotion/drake:tools/install/install.bzl
load("//tools:common.bzl", "dirname", "join_paths", "output_path")

InstallInfo = provider()

#==============================================================================
#BEGIN internal helpers

#------------------------------------------------------------------------------
def _workspace(ctx):
    """Compute name of current workspace."""

    # Check for override
    if hasattr(ctx.attr, "workspace"):
        if len(ctx.attr.workspace):
            return ctx.attr.workspace

    # Check for meaningful workspace_root
    workspace = ctx.label.workspace_root.split("/")[-1]
    if len(workspace):
        return workspace

    # If workspace_root is empty, assume we are the root workspace
    return ctx.workspace_name

def _rename(file_dest, rename):
    """Compute file name if file renamed."""
    if file_dest in rename:
        renamed = rename[file_dest]
        return join_paths(dirname(file_dest), renamed)
    return file_dest

def _depset_to_list(x):
    """Helper function to convert depset to list."""
    iter_list = x.to_list() if type(x) == "depset" else x
    return iter_list

#------------------------------------------------------------------------------
def _output_path(ctx, input_file, strip_prefix = [], warn_foreign = True):
    """Compute output path (without destination prefix) for install action.

    This computes the adjusted output path for an input file. It is the same as
    :func:`output_path`, but additionally handles files outside the current
    package.
    """

    # Try the current package first
    path = output_path(ctx, input_file, strip_prefix)
    if path != None:
        return path

    owner = input_file.owner
    if owner.workspace_name != "":
        dest = join_paths("third_party", owner.workspace_name, owner.package, input_file.basename)
    else:
        dest = join_paths(owner.package, input_file.basename)
    # print("Installing file {} ({}) which is not in current package".format(input_file.short_path, dest))
    return dest

#------------------------------------------------------------------------------
def _guess_files(target, candidates, scope, attr_name):
    if scope == "EVERYTHING":
        return candidates

    elif scope == "WORKSPACE":
        return [
            f
            for f in _depset_to_list(candidates)
            if target.label.workspace_root == f.owner.workspace_root
        ]

    elif scope == "PACKAGE":
        return [
            f
            for f in _depset_to_list(candidates)
            if (target.label.workspace_root == f.owner.workspace_root and
                target.label.package == f.owner.package)
        ]

    else:
        msg_fmt = "'install' given unknown '%s' value '%s'"
        fail(msg_fmt % (attr_name, scope), scope)

#------------------------------------------------------------------------------
def _install_action(
        ctx,
        artifact,
        dests,
        strip_prefixes = [],
        rename = {},
        warn_foreign = True):
    """Compute install action for a single file.

    This takes a single file artifact and returns the appropriate install
    action for the file. The parameters are the same as for
    :func:`_install_action`.
    """
    if type(dests) == "dict":
        dest = dests.get(artifact.extension, dests[None])
    else:
        dest = dests

    dest_replacements = (
        ("@WORKSPACE@", _workspace(ctx)),
        ("@PACKAGE@", ctx.label.package),
    )
    for old, new in dest_replacements:
        if old in dest:
            dest = dest.replace(old, new)

    if type(strip_prefixes) == "dict":
        strip_prefix = strip_prefixes.get(
            artifact.extension,
            strip_prefixes[None],
        )
    else:
        strip_prefix = strip_prefixes

    file_dest = join_paths(
        dest,
        _output_path(ctx, artifact, strip_prefix, warn_foreign),
    )
    file_dest = _rename(file_dest, rename)

    return struct(src = artifact, dst = file_dest)

#------------------------------------------------------------------------------
def _install_actions(
        ctx,
        file_labels,
        dests,
        strip_prefixes = [],
        excluded_files = [],
        rename = {},
        warn_foreign = True):
    """Compute install actions for files.

    This takes a list of labels (targets or files) and computes the install
    actions for the files owned by each label.

    Args:
        file_labels (:obj:`list` of :obj:`Label`): List of labels to install.
        dests (:obj:`str` or :obj:`dict` of :obj:`str` to :obj:`str`):
            Install destination. A :obj:`dict` may be given to supply a mapping
            of file extension to destination path. The :obj:`dict` must have an
            entry with the key ``None`` that is used as the default when there
            is no entry for the specific extension.
        strip_prefixes (:obj:`list` of :obj:`str` or :obj:`dict` of :obj:`list`
            of :obj:`str` to :obj:`str`): List of prefixes to strip from the
            input path before prepending the destination. A :obj:`dict` may be
            given to supply a mapping of file extension to list of prefixes to
            strip. The :obj:`dict` must have an entry with the key ``None``
            that is used as the default when there is no entry for the specific
            extension.
        excluded_files (:obj:`list` of :obj:`str`): List of files to exclude
            from installation.

    Returns:
        :obj:`list`: A list of install actions.
    """
    actions = []

    # Iterate over files. We expect a list of labels, which will have a 'files'
    # attribute that is a list of file artifacts. Thus this two-level loop.
    for f in file_labels:
        for a in _depset_to_list(f.files):
            # TODO(mwoehlke-kitware) refactor this to separate computing the
            # original relative path and the path with prefix(es) stripped,
            # then use the original relative path for both exclusions and
            # renaming.
            if _output_path(ctx, a, warn_foreign = False) in excluded_files:
                continue

            actions.append(
                _install_action(
                    ctx,
                    a,
                    dests,
                    strip_prefixes,
                    rename,
                    warn_foreign,
                ),
            )

    return actions

#------------------------------------------------------------------------------
# Compute install actions for a cc_library or cc_binary.
def _install_cc_actions(ctx, target):
    # Compute actions for target artifacts.
    dests = {
        "a": ctx.attr.archive_dest,
        "so": ctx.attr.library_dest,
        None: ctx.attr.runtime_dest,
    }
    strip_prefixes = {
        "a": ctx.attr.archive_strip_prefix,
        "so": ctx.attr.library_strip_prefix,
        None: ctx.attr.runtime_strip_prefix,
    }
    actions = _install_actions(
        ctx,
        [target],
        dests,
        strip_prefixes,
        rename = ctx.attr.rename,
    )

    mangled_solibs = [
        f
        for f in _depset_to_list(target.default_runfiles.files)
        if not f.is_source and target.label != f.owner
    ]

    if len(mangled_solibs):
        actions += _install_actions(
            ctx,
            [struct(files = mangled_solibs)],
            ctx.attr.mangled_library_dest,
            strip_prefixes = ctx.attr.mangled_library_strip_prefix,
            rename = ctx.attr.rename,
        )

    # Compute actions for guessed resource files.
    if ctx.attr.guess_data != "NONE":
        data = [
            f
            for f in _depset_to_list(target.data_runfiles.files)
            if f.is_source
        ]
        data = _guess_files(target, data, ctx.attr.guess_data, "guess_data")
        actions += _install_actions(
            ctx,
            [struct(files = data)],
            ctx.attr.data_dest,
            ctx.attr.data_strip_prefix,
            ctx.attr.guess_data_exclude,
            rename = ctx.attr.rename,
        )

    # Return computed actions.
    return actions

#------------------------------------------------------------------------------
# Compute install actions for a py_library or py_binary.
# TODO(jamiesnape): Install native shared libraries that the target may use.
def _install_py_actions(ctx, target):
    return _install_actions(
        ctx,
        [target],
        ctx.attr.py_dest,
        ctx.attr.py_strip_prefix,
        rename = ctx.attr.rename,
    )

#------------------------------------------------------------------------------
# Compute install actions for a script or an executable.
def _install_runtime_actions(ctx, target):
    return _install_actions(
        ctx,
        [target],
        ctx.attr.runtime_dest,
        ctx.attr.runtime_strip_prefix,
        rename = ctx.attr.rename,
    )

#------------------------------------------------------------------------------
# Generate install code for an install action.
def _install_code(action):
    return "install(%r, %r)" % (action.src.short_path, action.dst)

#BEGIN rules

#------------------------------------------------------------------------------
# Generate information to install "stuff". "Stuff" can be library or binary
# targets, or documentation files.
def _install_impl(ctx):
    actions = []
    rename = dict(ctx.attr.rename)

    # Collect install actions from dependencies.
    for d in ctx.attr.deps:
        actions += d[InstallInfo].install_actions
        rename.update(d[InstallInfo].rename)

    # Generate actions for data, docs and includes.
    actions += _install_actions(
        ctx,
        ctx.attr.docs,
        ctx.attr.doc_dest,
        strip_prefixes = ctx.attr.doc_strip_prefix,
        rename = rename,
    )

    actions += _install_actions(
        ctx,
        ctx.attr.data,
        ctx.attr.data_dest,
        strip_prefixes = ctx.attr.data_strip_prefix,
        rename = rename,
    )

    for t in ctx.attr.targets:
        # TODO(jwnimmer-tri): Raise an error if a target has testonly=1.
        if CcInfo in t:
            actions += _install_cc_actions(ctx, t)
            # linker_inputs = t[CcInfo].linking_context.linker_inputs

        elif PyInfo in t:
            actions += _install_py_actions(ctx, t)
        elif hasattr(t, "files_to_run") and t.files_to_run.executable:
            # Executable scripts copied from source directory.
            actions += _install_runtime_actions(ctx, t)

    # Generate code for install actions.
    script_actions = []
    installed_files = {}

    for a in actions:
        if not hasattr(a, "src"):
            fail("Action(dst={}) has no 'src' attribute".format(a.dst))
        src = a.src
        if a.dst not in installed_files:
            script_actions.append(_install_code(a))
            installed_files[a.dst] = src
        elif src != installed_files[a.dst]:
            orig = installed_files[a.dst]

            # Note(storypku):
            # Workaround for detected conflict betwen
            # <generated file external/local_config_cuda/cuda/cuda/lib/libcudart.so.11.0> and
            # <generated file _solib_local/_U@local_Uconfig_Ucuda_S_Scuda_Ccudart___Uexternal_Slocal_Uconfig_Ucuda_Scuda_Scuda_Slib/libcudart.so.11.0>
            # They share the same external workspace_root ("external/local_config_cuda") and package ("cuda")
            if src.basename != orig.basename or \
               src.owner.workspace_root != orig.owner.workspace_root or \
               src.owner.package != orig.owner.package:
                fail("Warning: Install conflict detected:\n" +
                     "\n  src1 = " + repr(orig) +
                     "\n  src2 = " + repr(src) +
                     "\n  dst = " + repr(a.dst))

    # Generate install script.
    # TODO(mwoehlke-kitware): Figure out a better way to generate this and run
    # it via Python than `#!/usr/bin/env python3`?
    ctx.actions.expand_template(
        template = ctx.executable.install_script_template,
        output = ctx.outputs.executable,
        substitutions = {"<<actions>>": "\n    ".join(script_actions)},
    )

    # Return actions.
    files = ctx.runfiles(
        files = [a.src for a in actions],
    )
    return [
        InstallInfo(
            install_actions = actions,
            rename = rename,
            installed_files = installed_files,
        ),
        DefaultInfo(runfiles = files),
    ]

# TODO(mwoehlke-kitware) default guess_data to PACKAGE when we have better
# default destinations.
_install_rule = rule(
    # Update buildifier-tables.json when this changes.
    attrs = {
        "deps": attr.label_list(providers = [InstallInfo]),
        "docs": attr.label_list(allow_files = True),
        "doc_dest": attr.string(default = "docs/@WORKSPACE@"),
        "doc_strip_prefix": attr.string_list(),
        "data": attr.label_list(allow_files = True),
        "data_dest": attr.string(default = "@PACKAGE@"),
        "data_strip_prefix": attr.string_list(),
        "guess_data": attr.string(default = "NONE"),
        "guess_data_exclude": attr.string_list(),
        "targets": attr.label_list(),
        "archive_dest": attr.string(default = "lib"),
        "archive_strip_prefix": attr.string_list(),
        "library_dest": attr.string(default = "@PACKAGE@"),
        "library_strip_prefix": attr.string_list(),
        "mangled_library_dest": attr.string(default = "lib"),
        "mangled_library_strip_prefix": attr.string_list(),
        "runtime_dest": attr.string(default = "bin"),
        "runtime_strip_prefix": attr.string_list(),
        "py_dest": attr.string(default = "lib/python"),
        "py_strip_prefix": attr.string_list(),
        "rename": attr.string_dict(),
        "install_script_template": attr.label(
            allow_files = True,
            executable = True,
            cfg = "target",
            default = Label("//tools/install:install.py.in"),
        ),
    },
    executable = True,
    implementation = _install_impl,
)

def install(tags = [], **kwargs):
    # (The documentation for this function is immediately below.)
    _install_rule(
        tags = tags + ["install"],
        **kwargs
    )

"""Generate installation information for various artifacts.

This generates installation information for various artifacts, including
documentation files, and targets (e.g. ``cc_binary``). By default,
the path of any files is included in the install destination.
See :rule:`install_files` for details.

Normally, you should not install files or targets from a workspace other than
the one invoking ``install``, and ``install`` will warn if asked to do so.

Destination paths may include the following placeholders:

* ``@WORKSPACE@``, replaced with the name of the workspace which invokes ``install``.

Note:
    By default, resource files to be installed must be explicitly
    listed. This is to work around an issue where Bazel does not appear to
    provide any mechanism to obtain the *direct* data files of a target,
    at rule instantiation. The ``guess_data`` parameter may be used to tell
    ``install`` to guess at what resource files will be installed.
    Possible values are:

    * ``"NONE"``: Only install files which are explicitly listed (i.e. by
      ``data``).
    * ``PACKAGE``:  For each target, install those files which are used by the
      target and owned by a target in the same package.
    * ``WORKSPACE``: For each target, install those files which are used by the
      target and owned by a target in the same workspace.
    * ``EVERYTHING``: Install all resources used by the target.

    The resource files considered are *all* resource files transitively used by
    the target. Any option other than ``NONE`` is also likely to install resource
    files used by other targets. In either case, this may result in the same file
    being considered for installation more than once.

    Note also that, because Bazel includes *all* run-time dependencies —
    including e.g. shared libraries — in a target's ``runfiles``, only *source*
    artifacts are considered when guessing resource files.

Args:
    deps: List of other install rules that this rule should include.
    docs: List of documentation files to install.
    doc_dest: Destination for documentation files
        (default = "docs/@WORKSPACE@").
    doc_strip_prefix: List of prefixes to remove from documentation paths.
    guess_data: See note.
    guess_data_exclude: List of resources found by ``guess_data`` to exclude
        from installation.
    data: List of (platform-independent) resource files to install.
    data_dest: Destination for resource files (default = "@PACKAGE@").
    data_strip_prefix: List of prefixes to remove from resource paths.
    targets: List of targets to install.
    archive_dest: Destination for static library targets (default = "lib").
    archive_strip_prefix: List of prefixes to remove from static library paths.
    library_dest: Destination for shared library targets (default = "lib").
    library_strip_prefix: List of prefixes to remove from shared library paths.
    mangled_library_dest: Destination for mangled shared library targets (default = "lib").
    mangled_library_strip_prefix: List of prefixes to remove from mangled shared library paths.
    runtime_dest: Destination for executable targets (default = "bin").
    runtime_strip_prefix: List of prefixes to remove from executable paths.
    py_dest: Destination for Python targets
        (default = "lib/python").
    py_strip_prefix: List of prefixes to remove from Python paths.
    rename: Mapping of install paths to alternate file names, used to rename
      files upon installation.
"""

#------------------------------------------------------------------------------
# Generate information to install files to specified destination.
def _install_files_impl(ctx):
    # Get path components.
    dest = ctx.attr.dest
    strip_prefix = ctx.attr.strip_prefix

    # Generate actions.
    actions = _install_actions(
        ctx,
        ctx.attr.files,
        dest,
        strip_prefix,
        rename = ctx.attr.rename,
    )

    # Return computed actions.
    return [InstallInfo(install_actions = actions, rename = ctx.attr.rename)]

_install_files_rule = rule(
    # Update buildifier-tables.json when this changes.
    attrs = {
        "dest": attr.string(mandatory = True),
        "files": attr.label_list(allow_files = True),
        "rename": attr.string_dict(),
        "strip_prefix": attr.string_list(),
    },
    implementation = _install_files_impl,
)

def install_files(tags = [], **kwargs):
    # (The documentation for this function is immediately below.)
    _install_files_rule(
        tags = tags + ["install"],
        **kwargs
    )

"""Generate installation information for files.

This generates installation information for a list of files. By default, any
path portion of the file as named is included in the install destination. For
example::

    install_files(
        dest = "docs",
        files = ["foo/bar.txt"],
        ...)

This will install ``bar.txt`` to the destination ``docs/foo``.

When this behavior is undesired, the ``strip_prefix`` parameter may be used to
specify a list of prefixes to be removed from input file paths before computing
the destination path. Stripping is not recursive; the first matching prefix
will be stripped. Prefixes support the single-glob (``*``) to match any single
path component, or the multi-glob (``**``) to match any number of path
components. Multi-glob matching is greedy. Globs may only be matched against
complete path components (e.g. ``a/*/`` is okay, but ``a*/`` is not treated as
a glob and will be matched literally). Due to Skylark limitations, at most one
``**`` may be matched.

Destination paths may include the placeholder ``@WORKSPACE``, which is replaced
with the name of the workspace which invokes ``install``.

``install_files`` has the same caveats regarding external files as
:func:`install`.

Args:
    dest: Destination for files.
    files: List of files to install.
    strip_prefix: List of prefixes to remove from input paths.
    rename: Mapping of install paths to alternate file names, used to rename
      files upon installation.
"""

#END rules
