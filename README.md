#

cloned from https://github.com/Hemlock5712/swerve-test.git

# CAN NOT PUSH TO MAIN !!
# MUST DO PULL REQUESTS AND MUST BE CHECKED WITH AT LEAST 2 PEOPLE

To make changes, you need a pull request.
First, checkout main and 

```
git checkout main
git pull
git checkout -b <your_branch_name>
```

Then make your changes, build and test them, and commit them to your branch.
After that, check if new code has been pushed to main.

```
git checkout main
git pull
git checkout <your_branch_name>
git merge main
```

If there are merge conflicts, fix them, then build and test. Then you are ready to push

```
git push
```

Open a pull request for your branch at https://github.com/gwhs/Robot2023-V2/pulls using the "New Pull Request" buttong

Specify the compare branch as `<your_branch_name>`, and target as `main`
Add a description of what was changed, and ask other software team members to review.

If a change needs must be made to `main`, an admin will need to change protection rules

