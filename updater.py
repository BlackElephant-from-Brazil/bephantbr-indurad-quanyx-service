import git


def updater():
    g = git.cmd.Git("https://github.com/guisartori/omega-ash.git")
    g.pull()
