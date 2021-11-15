Please adhere to the following guidelines for:

####  Reporting an issue

1. Title of the issue should describe the issue concisely and should not exceed 72 characters.
2. In the comment section, add the following details:
	* Commit ID of the code where the issue is observed.
	* Details related to toolchain, platform along with any other relevant information.
	* Frequency of occurrence.
	* Step-by-step procedure to reproduce the issue.
	* If the issue is observed with a specific stream, the test stream should be attached or a link for the same can be provided with public access.

####  Creating a pull request

1. A pull request can be created only if there is an issue report associated with it. 
   If issue is not reported yet, kindly report the issue in issues section by following the procedure mentioned in the above section.
2. Syntax for creating a branch to address any issue reported is `topic/issue-# (# is corresponds to issue number)`
3. Title of the pull request should be `Fix for issue-#`
4. Contents of pull request body should cover the following details:
	- Details of the bug
	- Details of the fix
	- Test done
	- Test results 
5. Ensure to add appropriate label.
6. Add reviewers and assign it to one of the reviewers

####  Writing a commit message

Follow the below rules in writing the commit message
1. Start with the subject: keep this short and simple
2. Start the body after a blank line from the subject
3. Wrap the contents in the body to 72 characters
4. Be descriptive in the body about:
	* Why the change is made
	* How the change addresses the issue
	* Any limitations of the change made
5. Add a section to describe the tests performed to validate the changes and the observed results.

#####  Sample Commit message

-----------------------------------------------

**Short summary**

Assign initial value to <some_variable> 

**More detailed explanation**

Initialising the local variable to store <some_purpose> in <some_function> function to

<intial_value> in order to ensure there is no uninitialised access. 

This issue was uncovered when tested with <problematic_stream> stream on <some_platform> platform.

This was resulting in glitches in the output.

Addressing this issue has resolved the glitches in the decoded output.
 
**Tests Done:**

Tested with conformance streams. Conformance criteria met. 

Tested with <problematic_stream> stream with which the issue was observed.

-------------------------------------------------

