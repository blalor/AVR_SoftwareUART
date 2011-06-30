.PHONY: test
test:
	make -C test

.PHONY: ci
ci:
	$(HOME)/devel/git_repos/simple-ci/bin/simple_ci.py . ./ci_wrapper.sh

