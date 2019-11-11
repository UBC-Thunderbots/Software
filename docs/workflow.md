# Workflow

## Table of Contents
* [Issue and Project Tracking](#issue-and-project-tracking)
    * [Issues](#issues)
* [Git Workflow](#git-workflow)
    * [Forking and Branching](#forking-and-branching)
    * [Creating a new Branch](#creating-a-new-branch)
        * [Why should you only create branches from "upstream/master"?](#why-should-you-only-create-branches-from-upstreammaster)
    * [Making Commits](#making-commits)
    * [Updating Your Branch and Resolving Conflicts](#updating-your-branch-and-resolving-conflicts)
    * [Formatting Your Code](#formatting-your-code)
    * [Pull Requests](#pull-requests)
    * [Reviewing Pull Requests](#reviewing-pull-requests)
* [Example Workflow](#example-workflow)
* [Testing](#testing)


## Issue and Project Tracking

We try keep our issue and project tracking fairly simple, to reduce the overhead associated with tracking all the information and to make it easier to follow. If you are unfamiliar with GitHub issues, [this article](https://guides.github.com/features/issues/) gives a good overview.

### Issues

We use issues to keep track of bugs in our system, and new features or enhancements we want to add. When creating a new issue, we have a simple "Task" template that can be filled out. We *strongly* recommend using the template since it provides guiding questions/headings to make sure we have all the necessary information in each issue.

*It is very important to give lots of detail and context when creating an issue. It is best to pretend you are writing the issue for someone who has not worked on the relevant part of the system before, and to leave a good enough explanation that someone with very little prior knowledge could get started. Sometimes issues get worked on many months after they were created, and we don't want to forget exactly what we wanted to do and why.*

In general if you find an issue with the system, first check with others on your team to make sure that this is indeed unintended behavior (you never know), and make sure that an issue has not already been created before you create a new one.  
  
The same goes for feature requests. Just make sure that whatever you want to say doesn't already exist in an issue.


## Git Workflow

### Forking and Branching

In general, we follow the Forking Workflow

* [What it is](https://www.atlassian.com/git/tutorials/comparing-workflows#forking-workflow)
* [How to use it](https://gist.github.com/Chaser324/ce0505fbed06b947d962)
* Instructions on obtaining your own Fork of our repository can be found on the [Getting Started](software-setup.md#getting-the-code) page.

### Creating a new Branch

For each Issue of project you are working on, you should have a separate branch. This helps keep work organized and separate.

**Branches should always be created from the latest code on the `master` branch of our main Software repository**. If you followed the steps in [Getting Started](software-setup.md), this will be `upstream/master`. Once this branch is created, you can push it to your fork and update it with commits until it is ready to merge. 

1. Navigate to the base folder of your Software repository: `cd path/to/the/repository/Software`
2. Make git aware of any new changes to `upstream` by running `git fetch upstream`
3. Create a new branch from `upstream/master` by running `git checkout -b your-branch-name upstream/master`
   1. Our branch naming convention is: `your_name/branch_name` (all lowercase, words separated by underscores). The branch name should be short and descriptive of the work being done on the branch.
   
**Example:** if you were working on a new navigation system using RRT and your name was "Bob" your branch name might look like: `bob/new_rrt_navigator`
4. You can now commit changes to this branch, and push them to your fork with `git push origin your_branch_name`

#### Why should you only create branches from "upstream/master"? 
Because we squash our commits when we merge Pull requests, a new commit with a new hash will be created, containing the multiple commits from the PR branch. Because the hashes are different, git will not recognize that the squashed commit and the series of commits that are inside the squashed commit contain the same changes, which can result in conflicts.

For example, lets pretend you have _branch A_, which was originally branched from `upstream/master`. You make a few commits and open a Pull Request. While you're waiting for the Pull Request to be reviewed and merged, you create a new branch, _branch B_, from _branch A_ to get a head start on a new feature. Eventually _branch A_ gets merged into `upstream/master`. Now you want to pull the latest changes from `upstream/master` into _branch B_ to make sure you have the latest code. git will treat the squashed commit that was merged from _branch A_'s Pull Request as a new change that needs to be merged, since _branch B_ will not have a commit with the same git hash. But _branch B_ already has these changes because it was created from branch A! This will cause massive merge conflicts that are nearly impossible to resolve cleanly.

tl;dr Always create new branches from upstream/master. Do not create branches from other feature branches.

### Making Commits

We don't impose any rules for how you should be committing code, just keep the following general rules in mind:

1. Commits should represent logical steps in your workflow. Avoid making commits too large, and try keep related changes together
2. Commit messages should give a good idea of the changes made. You don't have to go in-depth with technical details, but no one will know what you've done if your commit message is "fixed broken stuff"
3. Do not commit any non-code files such as images, videos, or generated files.

### Updating Your Branch and Resolving Conflicts
As you are working on your code on your branch and making commits, you'll want to update your branch with the latest code on `upstream/master` to make sure you're working with the latest code. This is important in case someone else merged new code that affects the code you're working on.

To do this, you have 2 options: rebase or merge. [What's the difference?](https://www.atlassian.com/git/tutorials/merging-vs-rebasing). 

Rebasing is generally recommended but requires slightly more knowledge of git. You can simply `git pull --rebase upstream master` to rebase your branch onto the latest `upstream/master`. If you do find you run into crazy conflicts, it might be worth aborting and attempting a merge.

To merge, simply run `git pull upstream master`. This _usually_ produces fewer conflicts than rebasing and is a safer option if you just want to get stuff working.

If you do rebase or merge and get conflicts, you'll need to resolve them manually. [See here for a quick tutorials on what conflicts are and how to resolve them](https://www.atlassian.com/git/tutorials/using-branches/merge-conflicts). Feel free to do this in your IDE or with whatever tool you are most comfortable with. Updating your branch often helps keep conflicts to a minimum, and when they do appear they are usually smaller. Ask for help if you're really stuck!

### Formatting Your Code
We use [clang-format](https://electronjs.org/docs/development/clang-format) to automatically format our code. Using an automatic tool helps keep things consistent across the codebase without developers having to change their personal style as they write. See the [code style guide](code-style-guide.md) for more information on exactly what it does.

To format the code, from the `Software` directory run `./clang_format/fix_formatting.sh -b master`. This will take the difference between your current branch and your local `master` branch, and run `clang-format` on those changes. The only reason we use the `-b master` option is because it's faster and only formats code you have modified. There is also a `-a` option that will try format the entire codebase, but it is slow and not recommended.

We recommend committing all your changes and then running the formatting script. This keeps the formatting changes separate from your actual changes. For the formatting changes, a simple commit message like "fixed formatting" is fine.

### Pull Requests

Pull Requests give us a chance to run our automated tests and review the code before it gets merged. This helps us make sure our code on `upstream/master` always compiles and is as bug-free as possible.

The code-review process gives us a chance ask questions or suggest improvements regarding a proposed change, so that the code is of the highest possible quality before being merged. It is also a good opportunity for others on the team to see what changes are being made, even if they are not involved in the project.

The Pull Request process usually looks like the following:
 
1. Make sure all the changes you want to make are pushed to a branch on your fork of the repository
2. Make sure you have [updated your branch](#updating-your-branch-and-resolving-conflicts) and [formatted your code](#formatting-your-code). This is to help make sure CI will pass.
3. From the main page of your fork of the Software repository, click on the "code" tab and then on the "branches" tab below.
4. Find the branch you want to open a Pull Request with and click "New pull request"
5. Make sure the target (base-fork) is the `UBC-Thunderbots/Software` repository with branch `master`
6. Give your Pull Request a short but descriptive title (the title should reflect the changes)
7. Fill out the pull request template. This includes things like a description of the changes, indicating which issues the Pull Request resolves, and indicating what testing has been done.
8. Add reviewers. This should be anyone that worked with you on the changes or is working on something that will be affected by the changes. Add your team lead and a few other members. Around 3-4 reviewers is a good number, but use your best judgement. Remember, these reviews also help give other team members an idea of the changes that are being made even if they aren't working on them.
    1. At least one "Code Owner" will need to review the change in order for it to be merged
9. Click "Create pull request"
10. Now the code can be reviewed. Respond to feedback given by your team members and make changes as necessary by pushing additional commits to your branch.
    1. **If you are a reviewer:**
       1. Look over the code, keeping an eye out for typos or bugs
       2. If you are having trouble understanding what a certain part of the code is doing, that's a great place to suggest adding additional comments!
       3. Remember you are critiquing someone's work. Give useful, constructive feedback and justify your thoughts, and don't be mean or degrading.
       4. During re-reviews (Pull Requests typically involve several rounds of changes and review), **it is your responsability to check that previously requested changes were made and mark the relevant discussions as "resolved"**. "Unresolved" discussions make a great checklist of things to check during a re-review.
       5. Mark the Pull Request as "Approved" when you think it looks good
    2. **If you are the recipient of the review (the PR creator):**
       1. **Make sure to reply to the PR comments as you address / fix issues**. This helps the reviewers know you have made a change without having to go check the code diffs to see if you made a change.
          1. Eg. Reply with "done" or "fixed" to comments as you address them
          2. Leave comments unresolved, let the reviewer resolve them.
       2. Don't be afraid to ask for clarification regarding changes or suggest alternatives if you don't agree with what was suggested. The reviewers and reviewee should work together to come up with the best solution.
       3. **Do not resolve conversations as you address them** (but make sure to leave a comment as mentioned above). That is the responsibility of the reviewers.
11. Make sure our automated tests with Travis CI are passing. There will be an indicator near the bottom of the Pull Request. If something fails, you can click on the links provided to get more information and debug the problems. More than likely, you'll just need to re-run clang-format on the code.
12. Once your Pull Request has been approved and the automated tests pass, you can merge the code. There will be a big 'merge" button at the bottom of the Pull Request with several options to choose from
    1. We only allow "Squash and merge". This is because it keep the commit history on `upstream/master` shorter and cleaner, without losing any context from the commit messages (since they are combined in the squashed commit. A squashed commit also makes it easier to revert and entire change/feature, rather than having to "know" the range of commits to revert.
13. That's it, your changes have been merged! You will be given the option to delete your remote branch. but are not required to do so. We recommend it since it will keep your fork cleaner, but you can do whatever you like.

*Remember, code reviews can be tough. As a reviewer, it can be very tricky to give useful constructive criticism without coming off as condescending or degrading (emotions are hard to express through text!). As the recipient of a code review, it might feel like you are being criticized too harshly and that your hard work is being attacked. Remember that these are your teammates, who are not trying to arbitrarily devalue your contributions but are trying to help make the code as good as possible, for the good of the team.*

### Reviewing Pull Requests

When reviewing pull requests, it can be really difficult to phrase comments in a way that doesn't come across as aggressive or mean. That said, it's really important that we strive to keep pull requests friendly and open, both for the health of everyone involved, and the effectiveness of the code review process. Here are two links that everyone reviewing a pull request should _thoroughly_ read before doing reviews:

[https://mtlynch.io/human-code-reviews-1/](https://mtlynch.io/human-code-reviews-1/) 

[https://mtlynch.io/human-code-reviews-2/](https://mtlynch.io/human-code-reviews-2/)


## Example Workflow

We find our workflow is best explained by walking through an example. We're assuming you have already cloned the repository and set up your git remotes. If not, check out our [Getting the Code](software-setup.md#getting-the-code) instructions first and then come back.

This example incorporates information from the previous sections on [Issue and Project Tracking](#issue-and-project-tracking), and [the Git Workflow](#git-workflow). Make sure you have read those sections first. This example skips over some of the smaller details.
  
We are also assuming all the work done here is in your fork of the repository.

Let's pretend our goalie strategy isn't that great. You have noticed that and suggested we improve it. Here's what your workflow would likely look like, from start to finish. We will pretend your name is Bob.

1. Create a new Issue for the goalie strategy if it doesn't already exist
   1. Let's pretend this is `Issue #23`
   2. Add relevant tags to your Issue. In this case, likely "Enhancement"
2. Create a new branch from `upstream/master`, called `bob/create_new_goalie_strategy`
   1. `git fetch upstream`
   2. `git checkout -b bob/create_new_goalie_strategy upstream/master`
3. Make your changes
   1. As you make changes and come across new information / challenges, it is good to update the Issue you are working on to document these new changes or requirements. Updating our progress on the ticket also helps other know how your work is going.
      1. `git commit -m "Improved the goalie's positioning during corner kicks, to block shots near the edge of the net"`
   2. Don't forget to push your changes to the branch on your fork occasionally, so you don't lose your work if something happens to your computer (it's happened to our team before)
4. Open a Pull Request to the master branch of the main Software repository. This will be a request to merge the branch `bob/create_new_goalie_strategy` from your fork, to the `master` branch of the main `Software` repository.
   1. The description should include `resolves #23`, so we know this Pull Request resolved your ticket
5. Discuss the changes with your reviewers and update the Pull Request by pushing additional commits to your branch
6. Once the Pull Request is approved and all CI checks pass, Squash and merge the changes
7. **Optional:** Delete your remote branch
8. Make sure your issue is marked as resolved. If you remembered to include `resolves #23` in your Pull Request description, this should have been done automatically for you, but it's good to double check.
9. Congratulations, you've now made changes to our main codebase!


## Testing

Testing is an integral part of our development process. If you are writing basically **any** code, it should be tested. If you feel like you can't test your piece of code, it's likely because it was written in a way that makes it difficult to test (which is a strong indicator for other problems, such as too much [coupling](https://en.wikipedia.org/wiki/Coupling_%28computer_programming%29)). _(An exception to this rule is any code that talks **directly** with hardware)_. For some examples of what happens when people don't test their code enough, see [here](http://outfresh.com/knowledge-base/6-famous-software-disasters-due-lack-testing/). How to run tests is explained in the [Getting Started Guide](getting-started.md/#building-and-running-the-code). 
