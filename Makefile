.PHONY: pkg

pkg:
	dpkg-buildpackage -us -uc -sd -b
clean:
	debian/rules clean
