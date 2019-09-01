---
description: A compilation of commonly asked questions and miscellaneous knowledge
---

# FAQ

<table>
  <thead>
    <tr>
      <th style="text-align:left">Question</th>
      <th style="text-align:left">Solution</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="text-align:left">My code completion / IntelliSense isn&apos;t working in CLion, but the
        project seems to be set up correctly and the code is building fine. What&apos;s
        wrong?</td>
      <td style="text-align:left">Build the project within CLion (rather than with catkin from the command
        line).</td>
    </tr>
    <tr>
      <td style="text-align:left">I&apos;m getting unexpected build failures, even after cleaning and rebuilding
        the project.</td>
      <td style="text-align:left">Clean the project again, delete the <code>build</code> and <code>devel</code> folders
        at the top of the repository, and rebuild again. The <code>build</code> and <code>devel</code> folders
        are part of the build process and can store some information that sometimes
        causes build problems, especially when doing complicated git operations.</td>
    </tr>
    <tr>
      <td style="text-align:left">It feels like the thing I&apos;m working on should have already been done
        somewhere. There&apos;s <em>no way</em> we don&apos;t already have a function
        or class for this already, right?</td>
      <td style="text-align:left">Ask around the team, especially your lead, if they know of something that
        already does what you need. If it feels like it should already have been
        done, it likely has and someone can point to to the right place to look.
        It&apos;s better to quickly ask than to duplicate work.</td>
    </tr>
    <tr>
      <td style="text-align:left">I can&apos;t source the setup script / I can&apos;t run <code>source devel/setup.sh</code> because
        the file/folder doesn&apos;t exist</td>
      <td style="text-align:left">The <code>devel</code> folder likely doesn&apos;t exist. This folder is
        generated when you run <code>catkin_make</code>, so simply run <code>catkin_make</code> from
        the <code>Software</code> folder (the base of the repo) to generate the folder.
        Then you should be able to run <code>source devel/setup.sh</code> successfully</td>
    </tr>
    <tr>
      <td style="text-align:left">I&apos;m trying to run a node using <code>rosrun</code> but I get an error
        similar to the following: <code>[ERROR] [1535424889.095662694]: [registerPublisher] Failed to contact master at [localhost:11311]. Retrying...</code>
      </td>
      <td style="text-align:left">You most likely forgot to run <code>roscore</code> first. Open a new terminal,
        run <code>roscore</code>, then try again</td>
    </tr>
    <tr>
      <td style="text-align:left">I tried to run a node using <code>rosrun</code>, but I get the following
        error: <code>[rospack] Error: package &apos;thunderbots&apos; not found</code>
      </td>
      <td style="text-align:left">
        <p>You most likely forgot to source the <code>setup.sh</code> file in your
          terminal session. Steps to fix:</p>
        <ul>
          <li>Open terminal and navigate to the root of the software repository</li>
          <li>run <code>source ./devel/setup.sh</code>
          </li>
          <li>If the <code>devel</code> folder doesn&apos;t exist, run <code>catkin_make</code> first</li>
          <li>Add a command to automatically source the <code>setup.sh</code> file to
            your <code>bashrc</code>, or re-run the <code>setup_software.sh</code> script.
            It should modify the <code>bashrc</code> file automatically.</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td style="text-align:left">I tried to run a launch file using roslaunch, but I get an error similar
        to the following: <code>RLException: [ai.launch] is neither a launch file in package [thunderbots] nor is [thunderbots] a launch file name</code>
      </td>
      <td style="text-align:left">You most likely forgot to source the <code>setup.sh</code> file in your
        terminal session. See the solution above.</td>
    </tr>
    <tr>
      <td style="text-align:left">
        <p>When running CMake in CLion, it fails with an error similar to: <code>ImportError: No module named catkin.environment_cache</code>
        </p>
        <p>or some other ros or catkin related error</p>
      </td>
      <td style="text-align:left">You most likely forgot to source the <code>setup.sh</code> file in your
        terminal before launching CLion. See the solution above for how to source
        the <code>setup.sh</code> file, and the run <code>clion</code> from the same
        terminal.</td>
    </tr>
  </tbody>
</table>