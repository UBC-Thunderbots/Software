@page editing_the_docs Editing the Docs 

[TOC]

A quick guide for editing and updating our documentation.

Most text can easily be edited with your favourite text editor, or directly in GitHub itself. There's really nothing special here.

#### Editing Diagrams
Editing diagrams is the tricky part about our documentation. All our diagrams live in the [images](images/) folder. These are all diagrams created by [draw.io](https://www.draw.io/?mode=github). Specifically, this is done with the "GitHub mode" of [draw.io](https://www.draw.io/?mode=github). This allows diagrams or images to be saved directly in a git repository, but still edited with [draw.io](https://www.draw.io/?mode=github) when needed.

We do this so that it's easier for everyone to update the diagrams and have them automatically saved back to our repository, rather than having to save the diagram file in Google Drive, give people access, and remember to export new images and replace them in the docs every time they need to be updated.

You can read a bit about how the GitHub mode of [draw.io](https://github.com/jgraph/drawio-github) works at https://github.com/jgraph/drawio-github.

The gist of it is that these image files are stored in our github repository with extra information embedded that allows [draw.io](https://www.draw.io/?mode=github) to edit them. It's the same data that would be saved as an `XML` file if you simply saved a [draw.io](https://www.draw.io/?mode=github) diagram instead of exporting it as an image.

The GitHub mode lets you connect your GitHub account, and then you can edit or create documents in any GitHub repository you have access to.

##### Creating a new Diagram
1. Open [draw.io](https://www.draw.io/?mode=github) and connect your GitHub account if necessary
2. Click `Create New Diagram`
3. Give the diagram a name with `.svg` as the ending. We prefer `.svg` files since they are vector images and therefore scale better without becoming pixelated.
4. Click `Create`. You'll be shown a dialogue with all the repositories your GitHub account has access to. Click on your fork of our `Software` repository. Eg. `MyGitHubUsername/Software`
5. Now you are basically in a filetree of the folders in the repository you selected
6. Change the branch you are looking at if necessary. At the top of the dialogue, click `master` and it will let you select a different branch if one exists.
7. Choose the folder where you want to store the new diagram. In our case that would be in `docs/images`
8. Once you have selected the folder you want to create the new diagram in, click `OK`
9. You'll now be taken to the regular [draw.io](https://github.com/jgraph/drawio-github) canvas where you can create and edit a diagram to your heart's content.

##### Editing a diagram
This is basically the same procedure as above, except click on `Open Existing Diagram` instead, and select the file you want to edit. **Make sure you're on the right branch.**

##### Saving Diagrams
Once you have edited a diagram, you will eventually want to save it. Because this is the GitHub mode of [draw.io](https://www.draw.io/?mode=github), this means you actually make a commit back to the repository to save the file.

To save your changes, you can either click `File -> Save`, or click the red "save now" box that appears if you have unsaved changed. In either case, a small dialogue will appear asking you to provide a commit message. Once you click `OK` a new commit will be made to the repository and branch you are editing the diagram in. The new diagram will appear anywhere it's linked without the need to do anything else!
