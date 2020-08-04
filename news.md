---
layout: page
title: News
permalink: /news/
---

{% for post in site.posts %}
{{ post.date | date: "%b %-d, %Y" }}:
# [{{ post.title }}]({{ post.url | prepend: site.baseurl }})
{% endfor %}
