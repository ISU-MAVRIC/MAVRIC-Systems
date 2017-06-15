# What is GitHub
You may know most (or all) of this, but some dont:
GitHub is, at its simplest, a file management system for projects.
It uses a tool called [Git](http://www.git-scm.com) to manage the files.
The newer side of git is a project management system. We'll walk through both.

### Terms
 - Github: This web site, includes the servers that host files, and the projects
 - Git: The tool/standard that versions and tracks files
 - Project: A tool on Github to track the status of work and provide visibility to progress.
 - Issue: Basically a thread that can be referenced by commits, sometimes the name is misleading,
 it is really just a discussion thread that can be assigned to projects and users, given labels, and helps manage things.
 - Repository/Repo: A grouped set of files/issues/projects that relate. MAVRIC-General is a repo for general design,
 and the other ones are for specific parts. We could have made it one, but it would tend to get less organized.
 - Organization: a github thing that groups users and repositories into a single area that has visibility accross itsself.
 - Commit:
   - [noun] A set of changes to the repo from a parent commit (initial commit has no parent)
   - [verb] To create a commit from some or all local changes
 - Pull: Gets changes to the repository from the server
   - It is advisable to commit or remove and local changes prior to pulling
 - Push: Upload commits to the server

## Git
Git is a tool to version changes to a set of files,
additionally git allows users to create isolated workspaces
that can later be combined with the rest of the project.
The most basic way to use github is to download their (desktop application)[desktop.github.com]
The desktop app will let you add repositories to your local machine
and helps with uploading changes.

## Projects
Projects are a way to track work. Each project has one or more columns, and eah column can have cards.
Cards are made from issues (and stay linked) which are in reposotories.
The projects are often used to track To Do/Doing/Done states for work items (e.g. #5)
Projects can be in a repo, an org, or a user account, and are visible only to the container they are in 
(I can't see you personal project boards, you can't see mine, but we can both see org boards, and any collaborator can see repo boards).

# Using Github
## Changing Files
to change a file in a repo, open github desktop, and select thr repo (or clone it if you haven't).
 1. Do a Pull (click sync). Always get the latest before working.
 2. Make your changes normally (outside of github desktop)
 3. Go back to github desktop and commit your changes (column near the left, enter in a title at least) see 2c985d4
 4. Push the commit to the server

## Adding an issue (work item, problem for discussion, suggestion, bug, ...)
 1. Go to the appropriate repository and click issues.
 2. Click New Issue

If you are making a work item, and it has sup-parts use a markdown task list:
start a new line with " - [ ] " and then the name this creates a checkbox item, a filled one looks like " - [x] "
 - [ ] Unchecked
 - [x] Checked
